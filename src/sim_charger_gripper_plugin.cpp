#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

namespace iii_drone::simulation
{

namespace
{

struct Box
{
  gz::math::Vector3d min{-0.2, -0.2, -0.3};
  gz::math::Vector3d max{0.2, 0.2, 0.1};

  bool Contains(const gz::math::Vector3d & point) const
  {
    return point.X() >= this->min.X() && point.X() <= this->max.X() &&
           point.Y() >= this->min.Y() && point.Y() <= this->max.Y() &&
           point.Z() >= this->min.Z() && point.Z() <= this->max.Z();
  }
};

struct SegmentClosestPoint
{
  gz::math::Vector3d point{gz::math::Vector3d::Zero};
  double distance{std::numeric_limits<double>::infinity()};
};

struct LatchSeatError
{
  double along{std::numeric_limits<double>::infinity()};
  double cross{std::numeric_limits<double>::infinity()};
};

gz::math::Vector3d ToLocal(
    const gz::math::Pose3d & frame_world_pose,
    const gz::math::Vector3d & world_point)
{
  return frame_world_pose.Rot().RotateVectorReverse(world_point - frame_world_pose.Pos());
}

std::string FindAsset(const std::string & uri)
{
  const std::string prefix = "model://";
  if (uri.rfind(prefix, 0) != 0) {
    return gz::common::findFile(uri);
  }

  const auto without_prefix = uri.substr(prefix.size());
  const auto slash = without_prefix.find('/');
  if (slash == std::string::npos) {
    return gz::common::findFile(uri);
  }

  const auto model = without_prefix.substr(0, slash);
  const auto relative = without_prefix.substr(slash + 1);
  const auto model_path = gz::common::findFile("model://" + model);
  if (model_path.empty()) {
    return {};
  }
  return model_path + "/" + relative;
}

SegmentClosestPoint ClosestPointOnSegment(
    const gz::math::Vector3d & point,
    const gz::math::Vector3d & segment_start,
    const gz::math::Vector3d & segment_end)
{
  const auto segment = segment_end - segment_start;
  const double length_squared = segment.SquaredLength();
  double t = 0.0;
  if (length_squared > 1.0e-12) {
    t = (point - segment_start).Dot(segment) / length_squared;
    t = std::clamp(t, 0.0, 1.0);
  }
  const auto closest = segment_start + segment * t;
  return {closest, (closest - point).Length()};
}

}  // namespace

class SimChargerGripperPlugin :
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  void Configure(
    const gz::sim::Entity & entity,
    const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager & ecm,
    gz::sim::EventManager &) override
  {
    this->model_entity_ = entity;
    this->model_ = gz::sim::Model(entity);

    if (sdf->HasElement("link_name")) {
      this->link_name_ = sdf->Get<std::string>("link_name");
    }
    if (sdf->HasElement("conductor_asset_uri")) {
      this->conductor_asset_uri_ = sdf->Get<std::string>("conductor_asset_uri");
    }
    if (sdf->HasElement("gripper_pose")) {
      this->gripper_pose_ = sdf->Get<gz::math::Pose3d>("gripper_pose");
    }
    if (sdf->HasElement("command_topic")) {
      this->command_topic_ = sdf->Get<std::string>("command_topic");
    }
    if (sdf->HasElement("state_topic")) {
      this->state_topic_ = sdf->Get<std::string>("state_topic");
    }
    if (sdf->HasElement("required_contact_duration_ms")) {
      this->required_contact_duration_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double, std::milli>(sdf->Get<double>("required_contact_duration_ms")));
    }
    if (sdf->HasElement("latch_local_point")) {
      this->latch_local_point_ = sdf->Get<gz::math::Vector3d>("latch_local_point");
    }
    if (sdf->HasElement("latch_radius")) {
      this->latch_radius_ = sdf->Get<double>("latch_radius");
    }
    if (sdf->HasElement("latch_along_tolerance")) {
      this->latch_along_tolerance_ = sdf->Get<double>("latch_along_tolerance");
    }
    if (sdf->HasElement("conductor_radius")) {
      this->conductor_radius_ = sdf->Get<double>("conductor_radius");
    }
    if (sdf->HasElement("support_stiffness")) {
      this->support_stiffness_ = sdf->Get<double>("support_stiffness");
    }
    if (sdf->HasElement("support_damping")) {
      this->support_damping_ = sdf->Get<double>("support_damping");
    }
    if (sdf->HasElement("support_force_limit")) {
      this->support_force_limit_ = sdf->Get<double>("support_force_limit");
    }

    this->ReadBox(sdf, "capture", this->capture_box_);

    if (!this->ResolveLink(ecm)) {
      gzerr << "SimChargerGripperPlugin could not resolve link [" << this->link_name_
            << "] on model entity [" << entity << "].\n";
      return;
    }
    this->link_.EnableVelocityChecks(ecm, true);

    if (!this->LoadConductors()) {
      gzerr << "SimChargerGripperPlugin failed to load conductors from ["
            << this->conductor_asset_uri_ << "].\n";
      return;
    }

    this->state_pub_ = this->transport_node_.Advertise<gz::msgs::StringMsg>(this->state_topic_);
    if (!this->transport_node_.Subscribe(
        this->command_topic_, &SimChargerGripperPlugin::OnCommand, this))
    {
      gzerr << "SimChargerGripperPlugin failed to subscribe to command topic ["
            << this->command_topic_ << "].\n";
      return;
    }

    gzmsg << "SimChargerGripperPlugin configured for link [" << this->link_name_
          << "] with " << this->conductors_.size() << " conductors.\n";
    this->configured_ = true;
  }

  void PreUpdate(
    const gz::sim::UpdateInfo & info,
    gz::sim::EntityComponentManager & ecm) override
  {
    if (!this->configured_ || info.paused) {
      return;
    }

    if (!this->ResolveLink(ecm)) {
      return;
    }

    const auto link_pose = this->link_.WorldPose(ecm);
    if (!link_pose) {
      return;
    }

    const auto gripper_world_pose = *link_pose * this->gripper_pose_;
    const auto detection = this->DetectConductor(gripper_world_pose);
    const bool capture_ready = detection.in_capture && detection.triggered;

    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      if (!this->armed_) {
        this->latched_ = false;
        this->trigger_started_ = std::chrono::steady_clock::duration::min();
      } else if (!this->latched_) {
        if (capture_ready) {
          if (this->trigger_started_ == std::chrono::steady_clock::duration::min()) {
            this->trigger_started_ = info.simTime;
          }

          if (info.simTime - this->trigger_started_ >= this->required_contact_duration_) {
            this->latched_ = true;
            this->latched_conductor_id_ = detection.conductor_id;
            this->latch_point_world_ = detection.closest_point_world;
          }
        } else {
          this->trigger_started_ = std::chrono::steady_clock::duration::min();
        }
      }

      if (this->latched_) {
        this->ApplySupportForce(ecm, gripper_world_pose);
      }
    }

    if (this->last_publish_time_ == std::chrono::steady_clock::duration::min() ||
      info.simTime - this->last_publish_time_ >= this->publish_period_)
    {
      this->PublishState(
        detection.in_capture, detection.triggered, capture_ready, detection.conductor_id, detection);
      this->last_publish_time_ = info.simTime;
    }
  }

private:
  struct Conductor
  {
    std::string id;
    std::vector<gz::math::Vector3d> samples_world;
  };

  struct Detection
  {
    bool in_capture{false};
    bool triggered{false};
    std::string conductor_id;
    gz::math::Vector3d closest_point_world{gz::math::Vector3d::Zero};
    gz::math::Vector3d closest_point_local{gz::math::Vector3d::Zero};
    gz::math::Vector3d nearest_point_local{gz::math::Vector3d::Zero};
    std::string nearest_conductor_id;
    double closest_distance{std::numeric_limits<double>::infinity()};
    double closest_cross_distance{std::numeric_limits<double>::infinity()};
    double closest_along_error{std::numeric_limits<double>::infinity()};
    double nearest_distance{std::numeric_limits<double>::infinity()};
  };

  LatchSeatError ComputeLatchSeatError(const gz::math::Vector3d & point_local) const
  {
    const auto delta = point_local - this->latch_local_point_;
    return {
      std::abs(delta.X()),
      std::hypot(delta.Y(), delta.Z())
    };
  }

  void ReadBox(
    const std::shared_ptr<const sdf::Element> & sdf,
    const std::string & prefix,
    Box & box) const
  {
    if (sdf->HasElement(prefix + "_min")) {
      box.min = sdf->Get<gz::math::Vector3d>(prefix + "_min");
    }
    if (sdf->HasElement(prefix + "_max")) {
      box.max = sdf->Get<gz::math::Vector3d>(prefix + "_max");
    }
  }

  bool ResolveLink(const gz::sim::EntityComponentManager & ecm)
  {
    if (this->link_entity_ != gz::sim::kNullEntity) {
      return true;
    }

    this->link_entity_ = this->model_.LinkByName(ecm, this->link_name_);
    if (this->link_entity_ == gz::sim::kNullEntity) {
      return false;
    }
    this->link_ = gz::sim::Link(this->link_entity_);
    return true;
  }

  bool LoadConductors()
  {
    const auto path = FindAsset(this->conductor_asset_uri_);
    if (path.empty()) {
      gzerr << "Could not resolve conductor asset URI [" << this->conductor_asset_uri_ << "].\n";
      return false;
    }

    YAML::Node root = YAML::LoadFile(path);
    if (!root["conductors"]) {
      return false;
    }

    for (const auto & conductor_node : root["conductors"]) {
      Conductor conductor;
      conductor.id = conductor_node["id"].as<std::string>();
      for (const auto & sample : conductor_node["samples"]) {
        conductor.samples_world.emplace_back(
          sample[0].as<double>(), sample[1].as<double>(), sample[2].as<double>());
      }
      if (conductor.samples_world.size() >= 2) {
        this->conductors_.push_back(std::move(conductor));
      }
    }

    return !this->conductors_.empty();
  }

  Detection DetectConductor(const gz::math::Pose3d & gripper_world_pose) const
  {
    Detection best;

    for (const auto & conductor : this->conductors_) {
      for (std::size_t i = 1; i < conductor.samples_world.size(); ++i) {
        const auto & start_world = conductor.samples_world[i - 1];
        const auto & end_world = conductor.samples_world[i];
        const auto start_local = ToLocal(gripper_world_pose, start_world);
        const auto end_local = ToLocal(gripper_world_pose, end_world);
        const auto nearest = ClosestPointOnSegment(
          gz::math::Vector3d::Zero, start_local, end_local);
        if (nearest.distance < best.nearest_distance) {
          best.nearest_distance = nearest.distance;
          best.nearest_point_local = nearest.point;
          best.nearest_conductor_id = conductor.id;
        }

        const auto seated = ClosestPointOnSegment(
          this->latch_local_point_, start_local, end_local);
        const bool segment_in_capture =
          this->capture_box_.Contains(start_local) ||
          this->capture_box_.Contains(end_local) ||
          this->capture_box_.Contains(seated.point);
        if (segment_in_capture) {
          best.in_capture = true;
        }

        const auto seated_error = this->ComputeLatchSeatError(seated.point);

        if (segment_in_capture && seated_error.cross < best.closest_cross_distance) {
          best.closest_distance = seated.distance;
          best.closest_cross_distance = seated_error.cross;
          best.closest_along_error = seated_error.along;
          best.closest_point_local = seated.point;
          best.closest_point_world =
            gripper_world_pose.Pos() + gripper_world_pose.Rot().RotateVector(seated.point);
          best.conductor_id = conductor.id;
        }

        const double latch_threshold = std::max(0.0, this->latch_radius_ - this->conductor_radius_);
        if (seated_error.cross <= latch_threshold &&
          seated_error.along <= this->latch_along_tolerance_ &&
          segment_in_capture &&
          seated_error.cross <= best.closest_cross_distance + 1.0e-9)
        {
          best.triggered = true;
        }
      }
    }

    return best;
  }

  void ApplySupportForce(
    gz::sim::EntityComponentManager & ecm,
    const gz::math::Pose3d & gripper_world_pose)
  {
    const auto linear_velocity =
      this->link_.WorldLinearVelocity(ecm).value_or(gz::math::Vector3d::Zero);
    const auto angular_velocity =
      this->link_.WorldAngularVelocity(ecm).value_or(gz::math::Vector3d::Zero);
    const auto target_gripper_world =
      this->latch_point_world_ - gripper_world_pose.Rot().RotateVector(this->latch_local_point_);
    const auto position_error = target_gripper_world - gripper_world_pose.Pos();
    const auto application_point_link =
      this->gripper_pose_.Pos() +
      this->gripper_pose_.Rot().RotateVector(this->latch_local_point_);
    const auto link_pose = this->link_.WorldPose(ecm);
    const auto application_offset_world =
      link_pose.has_value() ?
      link_pose->Rot().RotateVector(application_point_link) :
      gz::math::Vector3d::Zero;
    const auto point_velocity =
      linear_velocity + angular_velocity.Cross(application_offset_world);

    // A single compliant contact force at the gripper latch point lets gravity
    // rotate the vehicle naturally until its center of mass hangs below the cable.
    auto latch_force =
      position_error * this->support_stiffness_ -
      point_velocity * this->support_damping_;

    const double force_length = latch_force.Length();
    if (force_length > this->support_force_limit_) {
      latch_force *= this->support_force_limit_ / force_length;
    }

    this->link_.AddWorldForce(ecm, latch_force, application_point_link);
  }

  void OnCommand(const gz::msgs::StringMsg & msg)
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    if (msg.data() == "open") {
      this->armed_ = false;
      this->latched_ = false;
      this->latched_conductor_id_.clear();
      this->trigger_started_ = std::chrono::steady_clock::duration::min();
    } else if (msg.data() == "armed") {
      this->armed_ = true;
    }
  }

  void PublishState(
    const bool in_capture,
    const bool bottom_triggered,
    const bool capture_ready,
    const std::string & conductor_id,
    const Detection & detection)
  {
    bool armed = false;
    bool latched = false;
    std::string latched_conductor;
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      armed = this->armed_;
      latched = this->latched_;
      latched_conductor = this->latched_conductor_id_;
    }

    gz::msgs::StringMsg msg;
    std::ostringstream data;
    data << "armed=" << (armed ? "1" : "0")
         << ";capture=" << (in_capture ? "1" : "0")
         << ";bottom_triggered=" << (bottom_triggered ? "1" : "0")
         << ";triggered=" << (capture_ready ? "1" : "0")
         << ";latched=" << (latched ? "1" : "0")
         << ";conductor=" << (latched_conductor.empty() ? conductor_id : latched_conductor)
         << ";nearest_conductor=" << detection.nearest_conductor_id
         << ";nearest_local_x=" << detection.nearest_point_local.X()
         << ";nearest_local_y=" << detection.nearest_point_local.Y()
         << ";nearest_local_z=" << detection.nearest_point_local.Z()
         << ";nearest_distance=" << detection.nearest_distance
         << ";closest_local_x=" << detection.closest_point_local.X()
         << ";closest_local_y=" << detection.closest_point_local.Y()
         << ";closest_local_z=" << detection.closest_point_local.Z()
         << ";closest_distance=" << detection.closest_distance
         << ";closest_cross_distance=" << detection.closest_cross_distance
         << ";closest_along_error=" << detection.closest_along_error
         << ";latch_local_x=" << this->latch_local_point_.X()
         << ";latch_local_y=" << this->latch_local_point_.Y()
         << ";latch_local_z=" << this->latch_local_point_.Z()
         << ";latch_radius=" << this->latch_radius_
         << ";latch_along_tolerance=" << this->latch_along_tolerance_
         << ";conductor_radius=" << this->conductor_radius_
         << ";seated_error=" << detection.closest_cross_distance
         << ";";
    msg.set_data(data.str());
    this->state_pub_.Publish(msg);
  }

  gz::sim::Entity model_entity_{gz::sim::kNullEntity};
  gz::sim::Entity link_entity_{gz::sim::kNullEntity};
  gz::sim::Model model_{gz::sim::kNullEntity};
  gz::sim::Link link_;
  std::string link_name_{"base_link"};
  std::string conductor_asset_uri_{"model://hcaa_pylon_setup/conductors.yaml"};
  gz::math::Pose3d gripper_pose_{0.0, 0.0, 0.4, 1.57079632679, 0.0, 0.0};
  Box capture_box_{{-0.25, -0.25, -0.35}, {0.25, 0.25, 0.12}};
  std::string command_topic_{"/sim/charger_gripper/command"};
  std::string state_topic_{"/sim/charger_gripper/state"};
  std::chrono::steady_clock::duration required_contact_duration_{std::chrono::milliseconds(100)};
  std::chrono::steady_clock::duration trigger_started_{std::chrono::steady_clock::duration::min()};
  std::chrono::steady_clock::duration publish_period_{std::chrono::milliseconds(50)};
  std::chrono::steady_clock::duration last_publish_time_{std::chrono::steady_clock::duration::min()};
  gz::math::Vector3d latch_local_point_{0.0, 0.0, 0.0};
  double latch_radius_{0.05};
  double latch_along_tolerance_{0.18};
  double conductor_radius_{0.0};
  gz::transport::Node transport_node_;
  gz::transport::Node::Publisher state_pub_;
  std::vector<Conductor> conductors_;
  std::mutex mutex_;
  bool armed_{false};
  bool latched_{false};
  bool configured_{false};
  std::string latched_conductor_id_;
  gz::math::Vector3d latch_point_world_{gz::math::Vector3d::Zero};
  double support_stiffness_{600.0};
  double support_damping_{80.0};
  double support_force_limit_{250.0};
};

}  // namespace iii_drone::simulation

GZ_ADD_PLUGIN(
  iii_drone::simulation::SimChargerGripperPlugin,
  gz::sim::System,
  gz::sim::ISystemConfigure,
  gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
  iii_drone::simulation::SimChargerGripperPlugin,
  "iii_drone::simulation::SimChargerGripperPlugin")
