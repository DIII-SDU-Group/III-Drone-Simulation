#include <atomic>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <thread>

#include <gz/msgs/stringmsg.pb.h>
#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>

#include <iii_drone_interfaces/msg/charger_operating_mode.hpp>
#include <iii_drone_interfaces/msg/charger_status.hpp>
#include <iii_drone_interfaces/msg/gripper_status.hpp>
#include <iii_drone_interfaces/srv/gripper_command.hpp>
#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/sim_battery_charge.hpp>

namespace iii_drone::simulation
{

namespace
{

double Clamp(const double value, const double lo, const double hi)
{
  return std::max(lo, std::min(value, hi));
}

}  // namespace

class SimChargerGripperNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  SimChargerGripperNode()
  : rclcpp_lifecycle::LifecycleNode("charger_gripper", "/payload/charger_gripper")
  {
    this->declare_parameter<double>("status_publish_rate_hz", 50.0);
    this->declare_parameter<std::string>("px4_battery_status_topic", "/fmu/out/battery_status");
    this->declare_parameter<std::string>("px4_battery_charge_topic", "/fmu/in/sim_battery_charge");
    this->declare_parameter<double>("px4_battery_status_timeout_s", 1.0);
    this->declare_parameter<double>("fallback_battery_voltage_v", 24.0);
    this->declare_parameter<bool>("require_px4_battery_for_charging", true);
    this->declare_parameter<double>("nominal_charging_power_w", 150.0);
    this->declare_parameter<double>("max_charging_power_w", 250.0);
    this->declare_parameter<double>("charging_power_noise_std_w", 5.0);
    this->declare_parameter<double>("charging_ramp_time_s", 1.0);
    this->declare_parameter<double>("trickle_power_w", 0.0);
    this->declare_parameter<double>("fully_charged_voltage_v", 25.2);
    this->declare_parameter<double>("fully_charged_remaining_pct", 0.98);
    this->declare_parameter<std::string>("gz_command_topic", "/sim/charger_gripper/command");
    this->declare_parameter<std::string>("gz_state_topic", "/sim/charger_gripper/state");
    this->declare_parameter<double>("gz_state_timeout_s", 1.0);

    this->status_publish_rate_hz_ = this->get_parameter("status_publish_rate_hz").as_double();
    this->px4_battery_status_timeout_s_ =
      this->get_parameter("px4_battery_status_timeout_s").as_double();
    this->fallback_battery_voltage_v_ =
      this->get_parameter("fallback_battery_voltage_v").as_double();
    this->require_px4_battery_for_charging_ =
      this->get_parameter("require_px4_battery_for_charging").as_bool();
    this->nominal_charging_power_w_ =
      this->get_parameter("nominal_charging_power_w").as_double();
    this->max_charging_power_w_ =
      this->get_parameter("max_charging_power_w").as_double();
    this->charging_power_noise_std_w_ =
      this->get_parameter("charging_power_noise_std_w").as_double();
    this->charging_ramp_time_s_ =
      this->get_parameter("charging_ramp_time_s").as_double();
    this->trickle_power_w_ = this->get_parameter("trickle_power_w").as_double();
    this->fully_charged_voltage_v_ =
      this->get_parameter("fully_charged_voltage_v").as_double();
    this->fully_charged_remaining_pct_ =
      this->get_parameter("fully_charged_remaining_pct").as_double();
    this->px4_battery_status_topic_ =
      this->get_parameter("px4_battery_status_topic").as_string();
    this->px4_battery_charge_topic_ =
      this->get_parameter("px4_battery_charge_topic").as_string();
    this->gz_command_topic_ = this->get_parameter("gz_command_topic").as_string();
    this->gz_state_topic_ = this->get_parameter("gz_state_topic").as_string();
    this->gz_state_timeout_s_ = this->get_parameter("gz_state_timeout_s").as_double();
    this->last_update_time_ = this->now();
  }

private:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  enum class State
  {
    Open,
    ArmedToClose,
    Latched,
    Charging,
    FullyCharged,
  };

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    auto status_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    auto px4_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

    this->gripper_status_pub_ =
      this->create_publisher<iii_drone_interfaces::msg::GripperStatus>("gripper_status", status_qos);
    this->battery_voltage_pub_ =
      this->create_publisher<std_msgs::msg::Float32>("battery_voltage", status_qos);
    this->charging_power_pub_ =
      this->create_publisher<std_msgs::msg::Float32>("charging_power", status_qos);
    this->charger_status_pub_ =
      this->create_publisher<iii_drone_interfaces::msg::ChargerStatus>("charger_status", status_qos);
    this->charger_operating_mode_pub_ =
      this->create_publisher<iii_drone_interfaces::msg::ChargerOperatingMode>(
        "charger_operating_mode", status_qos);
    this->px4_charge_pub_ =
      this->create_publisher<px4_msgs::msg::SimBatteryCharge>(
        this->px4_battery_charge_topic_, px4_qos);
    this->sim_state_pub_ = this->create_publisher<std_msgs::msg::String>("sim_state", status_qos);

    this->px4_battery_sub_ =
      this->create_subscription<px4_msgs::msg::BatteryStatus>(
        this->px4_battery_status_topic_, px4_qos,
        [this](const px4_msgs::msg::BatteryStatus::SharedPtr msg) {
          this->last_battery_msg_ = *msg;
          this->last_battery_time_ = this->now();
          this->have_battery_ = true;
        });

    this->gripper_command_srv_ =
      this->create_service<iii_drone_interfaces::srv::GripperCommand>(
        "gripper_command",
        [this](
          const std::shared_ptr<iii_drone_interfaces::srv::GripperCommand::Request> request,
          std::shared_ptr<iii_drone_interfaces::srv::GripperCommand::Response> response) {
          this->HandleGripperCommand(request, response);
        });

    this->state_ = State::Open;
    this->latched_ = false;
    this->triggered_ = false;
    this->last_logged_armed_ = false;
    this->last_logged_triggered_ = false;
    this->last_logged_latched_ = false;
    this->last_gz_log_steady_ns_ = 0;
    this->charging_power_w_ = 0.0;
    this->last_gz_state_steady_ns_.store(0);
    this->last_update_time_ = this->now();
    this->status_publish_count_ = 0;
    RCLCPP_INFO(this->get_logger(), "Simulated charger gripper configured.");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(this->get_logger(), "Activating simulated charger gripper.");
    this->gripper_status_pub_->on_activate();
    this->battery_voltage_pub_->on_activate();
    this->charging_power_pub_->on_activate();
    this->charger_status_pub_->on_activate();
    this->charger_operating_mode_pub_->on_activate();
    this->px4_charge_pub_->on_activate();

    const auto period = std::chrono::duration<double>(
      1.0 / std::max(1.0, this->status_publish_rate_hz_));
    this->status_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { this->PublishStatus(); });
    this->last_update_time_ = this->now();
    this->StartGzBridge();
    RCLCPP_INFO(
      this->get_logger(), "Simulated charger gripper active at %.2f Hz.",
      this->status_publish_rate_hz_);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    this->status_timer_.reset();
    {
      std::lock_guard<std::mutex> lock(this->gz_state_mutex_);
      this->state_ = State::Open;
      this->latched_ = false;
      this->triggered_ = false;
      this->charging_power_w_ = 0.0;
    }
    this->last_gz_state_steady_ns_.store(0);
    this->SendGzCommand("open");

    this->gripper_status_pub_->on_deactivate();
    this->battery_voltage_pub_->on_deactivate();
    this->charging_power_pub_->on_deactivate();
    this->charger_status_pub_->on_deactivate();
    this->charger_operating_mode_pub_->on_deactivate();
    this->px4_charge_pub_->on_deactivate();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    this->status_timer_.reset();
    this->gripper_command_srv_.reset();
    this->px4_battery_sub_.reset();
    this->gripper_status_pub_.reset();
    this->battery_voltage_pub_.reset();
    this->charging_power_pub_.reset();
    this->charger_status_pub_.reset();
    this->charger_operating_mode_pub_.reset();
    this->px4_charge_pub_.reset();
    {
      std::lock_guard<std::mutex> lock(this->gz_state_mutex_);
      this->state_ = State::Open;
      this->latched_ = false;
      this->triggered_ = false;
      this->charging_power_w_ = 0.0;
    }
    this->last_gz_state_steady_ns_.store(0);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
  {
    this->status_timer_.reset();
    this->charging_power_w_ = 0.0;
    return CallbackReturn::SUCCESS;
  }

  void HandleGripperCommand(
    const std::shared_ptr<iii_drone_interfaces::srv::GripperCommand::Request> request,
    std::shared_ptr<iii_drone_interfaces::srv::GripperCommand::Response> response)
  {
    if (request->gripper_command ==
      iii_drone_interfaces::srv::GripperCommand::Request::GRIPPER_COMMAND_OPEN)
    {
      {
        std::lock_guard<std::mutex> lock(this->gz_state_mutex_);
        this->state_ = State::Open;
        this->latched_ = false;
        this->triggered_ = false;
        this->charging_power_w_ = 0.0;
      }
      RCLCPP_INFO(this->get_logger(), "Gripper command OPEN: clearing latch and disarming Gazebo latch.");
      response->gripper_command_response =
        this->SendGzCommand("open") ?
        iii_drone_interfaces::srv::GripperCommand::Response::GRIPPER_COMMAND_RESPONSE_SUCCESS :
        iii_drone_interfaces::srv::GripperCommand::Response::GRIPPER_COMMAND_RESPONSE_ERROR;
      return;
    }

    if (request->gripper_command ==
      iii_drone_interfaces::srv::GripperCommand::Request::GRIPPER_COMMAND_CLOSE)
    {
      {
        std::lock_guard<std::mutex> lock(this->gz_state_mutex_);
        if (this->state_ == State::Open) {
          this->state_ = State::ArmedToClose;
        }
      }
      RCLCPP_INFO(this->get_logger(), "Gripper command CLOSE: arming Gazebo latch.");
      response->gripper_command_response =
        this->SendGzCommand("armed") ?
        iii_drone_interfaces::srv::GripperCommand::Response::GRIPPER_COMMAND_RESPONSE_SUCCESS :
        iii_drone_interfaces::srv::GripperCommand::Response::GRIPPER_COMMAND_RESPONSE_ERROR;
      return;
    }

    response->gripper_command_response =
      iii_drone_interfaces::srv::GripperCommand::Response::GRIPPER_COMMAND_RESPONSE_INVALID_COMMAND;
  }

  void OnGzState(const gz::msgs::StringMsg & msg)
  {
    const auto now_steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
    this->last_gz_state_steady_ns_.store(now_steady_ns);
    const auto & data = msg.data();
    if (this->sim_state_pub_) {
      std_msgs::msg::String sim_state;
      sim_state.data = data;
      this->sim_state_pub_->publish(sim_state);
    }

    const bool triggered = data.find("triggered=1") != std::string::npos;
    const bool latched = data.find("latched=1") != std::string::npos;
    const bool armed = data.find("armed=1") != std::string::npos;
    std::string conductor_id;
    const auto conductor_pos = data.find("conductor=");
    if (conductor_pos != std::string::npos) {
      const auto end = data.find(';', conductor_pos);
      conductor_id = data.substr(
        conductor_pos + std::string("conductor=").size(),
        end == std::string::npos ? std::string::npos :
        end - conductor_pos - std::string("conductor=").size());
    }

    bool state_changed = false;
    bool should_log = false;
    {
      std::lock_guard<std::mutex> lock(this->gz_state_mutex_);
      this->triggered_ = triggered;
      this->latched_ = latched;
      this->latched_conductor_id_ = conductor_id;
      if (this->latched_ && this->state_ == State::ArmedToClose) {
        this->state_ = State::Latched;
      }
      state_changed =
        armed != this->last_logged_armed_ ||
        triggered != this->last_logged_triggered_ ||
        latched != this->last_logged_latched_;
      constexpr int64_t log_interval_ns = 1000000000LL;
      should_log = state_changed ||
        (armed && !latched && now_steady_ns - this->last_gz_log_steady_ns_ >= log_interval_ns);
      if (should_log) {
        this->last_gz_log_steady_ns_ = now_steady_ns;
        this->last_logged_armed_ = armed;
        this->last_logged_triggered_ = triggered;
        this->last_logged_latched_ = latched;
      }
    }

    if (should_log) {
      RCLCPP_INFO(
        this->get_logger(),
        "Sim gripper state: armed=%s triggered=%s latched=%s conductor=%s nearest=(%s,%s,%s) closest=(%s,%s,%s) closest_distance=%s latch_radius=%s seated_error=%s raw=[%s]",
        armed ? "true" : "false",
        triggered ? "true" : "false",
        latched ? "true" : "false",
        this->ExtractField(data, "conductor").c_str(),
        this->ExtractField(data, "nearest_local_x").c_str(),
        this->ExtractField(data, "nearest_local_y").c_str(),
        this->ExtractField(data, "nearest_local_z").c_str(),
        this->ExtractField(data, "closest_local_x").c_str(),
        this->ExtractField(data, "closest_local_y").c_str(),
        this->ExtractField(data, "closest_local_z").c_str(),
        this->ExtractField(data, "closest_distance").c_str(),
        this->ExtractField(data, "latch_radius").c_str(),
        this->ExtractField(data, "seated_error").c_str(),
        data.c_str());
    }
  }

  std::string ExtractField(const std::string & data, const std::string & key) const
  {
    const std::string token = key + "=";
    const auto begin = data.find(token);
    if (begin == std::string::npos) {
      return {};
    }
    const auto value_begin = begin + token.size();
    const auto end = data.find(';', value_begin);
    return data.substr(value_begin, end == std::string::npos ? std::string::npos : end - value_begin);
  }

  void StartGzBridge()
  {
    if (this->gz_bridge_ready_.load() || this->gz_bridge_initializing_.exchange(true)) {
      return;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Starting simulated charger gripper Gazebo bridge: command=[%s] state=[%s]",
      this->gz_command_topic_.c_str(), this->gz_state_topic_.c_str());

    std::thread(
      [this]() {
        auto publisher = this->gz_node_.Advertise<gz::msgs::StringMsg>(this->gz_command_topic_);
        if (!publisher) {
          RCLCPP_ERROR(
            this->get_logger(), "Failed to advertise Gazebo charger/gripper command topic [%s]",
            this->gz_command_topic_.c_str());
          this->gz_bridge_initializing_.store(false);
          return;
        }

        if (!this->gz_node_.Subscribe(
            this->gz_state_topic_, &SimChargerGripperNode::OnGzState, this))
        {
          RCLCPP_ERROR(
            this->get_logger(), "Failed to subscribe to Gazebo charger/gripper state topic [%s]",
            this->gz_state_topic_.c_str());
          this->gz_bridge_initializing_.store(false);
          return;
        }

        {
          std::lock_guard<std::mutex> lock(this->gz_command_mutex_);
          this->gz_command_pub_ = publisher;
        }
        this->gz_bridge_ready_.store(true);
        this->gz_bridge_initializing_.store(false);
        RCLCPP_INFO(this->get_logger(), "Simulated charger gripper Gazebo bridge ready.");
      }).detach();
  }

  bool SendGzCommand(const std::string & command)
  {
    gz::transport::Node::Publisher publisher;
    {
      std::lock_guard<std::mutex> lock(this->gz_command_mutex_);
      if (!this->gz_bridge_ready_.load() || !this->gz_command_pub_) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Cannot send Gazebo charger/gripper command [%s]: Gazebo bridge is not ready",
          command.c_str());
        this->StartGzBridge();
        return false;
      }
      publisher = this->gz_command_pub_;
    }

    const auto logger = this->get_logger();
    std::thread(
      [publisher, command, logger]() mutable {
        gz::msgs::StringMsg msg;
        msg.set_data(command);
        if (!publisher.Publish(msg)) {
          RCLCPP_ERROR(logger, "Failed to publish Gazebo charger/gripper command [%s]", command.c_str());
        }
      }).detach();
    return true;
  }

  bool BatteryFresh() const
  {
    if (!this->have_battery_) {
      return false;
    }
    return (this->now() - this->last_battery_time_).seconds() <=
      this->px4_battery_status_timeout_s_;
  }

  double BatteryVoltage() const
  {
    if (this->BatteryFresh() && this->last_battery_msg_.voltage_v > 0.0f) {
      return this->last_battery_msg_.voltage_v;
    }
    return this->fallback_battery_voltage_v_;
  }

  bool BatteryFull() const
  {
    if (this->BatteryFresh()) {
      if (this->last_battery_msg_.remaining >= this->fully_charged_remaining_pct_) {
        return true;
      }
      if (this->last_battery_msg_.voltage_v >= this->fully_charged_voltage_v_) {
        return true;
      }
    }
    return this->BatteryVoltage() >= this->fully_charged_voltage_v_;
  }

  bool LatchConfirmed() const
  {
    std::lock_guard<std::mutex> lock(this->gz_state_mutex_);
    return this->LatchConfirmedLocked();
  }

  bool LatchConfirmedLocked() const
  {
    if (!this->latched_) {
      return false;
    }
    const auto last_state_ns = this->last_gz_state_steady_ns_.load();
    if (last_state_ns <= 0) {
      return false;
    }
    const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
    return static_cast<double>(now_ns - last_state_ns) / 1e9 <= this->gz_state_timeout_s_;
  }

  void UpdateStateAndPower()
  {
    std::lock_guard<std::mutex> lock(this->gz_state_mutex_);
    const auto now = this->now();
    const auto dt = std::max(0.001, (now - this->last_update_time_).seconds());
    this->last_update_time_ = now;

    const bool latch_confirmed = this->LatchConfirmedLocked();
    if (!latch_confirmed && (this->state_ == State::Latched ||
        this->state_ == State::Charging || this->state_ == State::FullyCharged))
    {
      this->state_ = State::ArmedToClose;
    }

    if (latch_confirmed && this->state_ == State::ArmedToClose) {
      this->state_ = State::Latched;
    }

    if (latch_confirmed && this->state_ == State::Latched) {
      this->state_ = this->BatteryFull() ? State::FullyCharged : State::Charging;
    }

    if (this->state_ == State::Charging && this->BatteryFull()) {
      this->state_ = State::FullyCharged;
    }

    if (this->state_ == State::FullyCharged && !this->BatteryFull()) {
      this->state_ = State::Charging;
    }

    if (!latch_confirmed) {
      this->charging_power_w_ = 0.0;
      return;
    }

    double target_power_w = 0.0;
    if (this->state_ == State::Charging) {
      if (!this->require_px4_battery_for_charging_ || this->BatteryFresh()) {
        target_power_w = this->nominal_charging_power_w_;
      }
    } else if (this->state_ == State::FullyCharged) {
      target_power_w = this->trickle_power_w_;
    }

    const double ramp_time = std::max(1e-3, this->charging_ramp_time_s_);
    const double max_delta = this->max_charging_power_w_ * dt / ramp_time;
    const double delta = Clamp(target_power_w - this->charging_power_w_, -max_delta, max_delta);
    this->charging_power_w_ += delta;

    if (this->charging_power_w_ > 0.0 && this->charging_power_noise_std_w_ > 0.0) {
      this->charging_power_w_ += this->noise_(this->random_) * this->charging_power_noise_std_w_;
    }

    this->charging_power_w_ = Clamp(this->charging_power_w_, 0.0, this->max_charging_power_w_);
  }

  void PublishStatus()
  {
    this->UpdateStateAndPower();

    State state;
    bool latched;
    double charging_power_w;
    {
      std::lock_guard<std::mutex> lock(this->gz_state_mutex_);
      state = this->state_;
      latched = this->latched_;
      charging_power_w = this->charging_power_w_;
    }

    iii_drone_interfaces::msg::GripperStatus gripper_status;
    const bool latch_confirmed = this->LatchConfirmed();
    gripper_status.gripper_status =
      latch_confirmed ?
      iii_drone_interfaces::msg::GripperStatus::GRIPPER_STATUS_CLOSED :
      iii_drone_interfaces::msg::GripperStatus::GRIPPER_STATUS_OPEN;
    this->gripper_status_pub_->publish(gripper_status);

    std_msgs::msg::Float32 battery_voltage;
    battery_voltage.data = static_cast<float>(this->BatteryVoltage());
    this->battery_voltage_pub_->publish(battery_voltage);

    std_msgs::msg::Float32 charging_power;
    charging_power.data = static_cast<float>(charging_power_w);
    this->charging_power_pub_->publish(charging_power);

    iii_drone_interfaces::msg::ChargerStatus charger_status;
    if (state == State::FullyCharged) {
      charger_status.charger_status =
        iii_drone_interfaces::msg::ChargerStatus::CHARGER_STATUS_FULLY_CHARGED;
    } else if (state == State::Charging && charging_power_w > 0.1) {
      charger_status.charger_status =
        iii_drone_interfaces::msg::ChargerStatus::CHARGER_STATUS_CHARGING;
    } else {
      charger_status.charger_status =
        iii_drone_interfaces::msg::ChargerStatus::CHARGER_STATUS_DISABLED;
    }
    this->charger_status_pub_->publish(charger_status);

    iii_drone_interfaces::msg::ChargerOperatingMode mode;
    mode.operating_mode = latch_confirmed ?
      iii_drone_interfaces::msg::ChargerOperatingMode::OPERATING_MODE_1 :
      iii_drone_interfaces::msg::ChargerOperatingMode::OPERATING_MODE_OPEN;
    this->charger_operating_mode_pub_->publish(mode);

    px4_msgs::msg::SimBatteryCharge charge;
    charge.timestamp = this->now().nanoseconds() / 1000;
    charge.charging_enabled = latch_confirmed && charging_power_w > 0.1;
    charge.charging_power_w = charge.charging_enabled ?
      static_cast<float>(charging_power_w) : 0.0f;
    this->px4_charge_pub_->publish(charge);

    ++this->status_publish_count_;
    if (this->status_publish_count_ <= 3) {
      RCLCPP_INFO(
        this->get_logger(), "Published simulated gripper status sample %lu: latched=%s state=%u",
        static_cast<unsigned long>(this->status_publish_count_),
        latched ? "true" : "false",
        static_cast<unsigned>(gripper_status.gripper_status));
    }
  }

  State state_{State::Open};
  bool triggered_{false};
  bool latched_{false};
  bool last_logged_armed_{false};
  bool last_logged_triggered_{false};
  bool last_logged_latched_{false};
  int64_t last_gz_log_steady_ns_{0};
  std::string latched_conductor_id_;
  mutable std::mutex gz_state_mutex_;
  double status_publish_rate_hz_{50.0};
  double px4_battery_status_timeout_s_{1.0};
  double fallback_battery_voltage_v_{24.0};
  bool require_px4_battery_for_charging_{true};
  double nominal_charging_power_w_{150.0};
  double max_charging_power_w_{250.0};
  double charging_power_noise_std_w_{5.0};
  double charging_ramp_time_s_{1.0};
  double trickle_power_w_{0.0};
  double fully_charged_voltage_v_{25.2};
  double fully_charged_remaining_pct_{0.98};
  double charging_power_w_{0.0};
  bool have_battery_{false};
  std::uint64_t status_publish_count_{0};
  px4_msgs::msg::BatteryStatus last_battery_msg_{};
  rclcpp::Time last_battery_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};
  std::string px4_battery_status_topic_;
  std::string px4_battery_charge_topic_;
  std::string gz_command_topic_;
  std::string gz_state_topic_;
  double gz_state_timeout_s_{1.0};
  std::atomic<int64_t> last_gz_state_steady_ns_{0};
  gz::transport::Node gz_node_;
  gz::transport::Node::Publisher gz_command_pub_;
  std::mutex gz_command_mutex_;
  std::atomic_bool gz_bridge_initializing_{false};
  std::atomic_bool gz_bridge_ready_{false};
  std::mt19937 random_{std::random_device{}()};
  std::normal_distribution<double> noise_{0.0, 1.0};

  rclcpp_lifecycle::LifecyclePublisher<iii_drone_interfaces::msg::GripperStatus>::SharedPtr
    gripper_status_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float32>::SharedPtr battery_voltage_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float32>::SharedPtr charging_power_pub_;
  rclcpp_lifecycle::LifecyclePublisher<iii_drone_interfaces::msg::ChargerStatus>::SharedPtr
    charger_status_pub_;
  rclcpp_lifecycle::LifecyclePublisher<iii_drone_interfaces::msg::ChargerOperatingMode>::SharedPtr
    charger_operating_mode_pub_;
  rclcpp_lifecycle::LifecyclePublisher<px4_msgs::msg::SimBatteryCharge>::SharedPtr px4_charge_pub_;
  rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr px4_battery_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sim_state_pub_;
  rclcpp::Service<iii_drone_interfaces::srv::GripperCommand>::SharedPtr gripper_command_srv_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace iii_drone::simulation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<iii_drone::simulation::SimChargerGripperNode>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
