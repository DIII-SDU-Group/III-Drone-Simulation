#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <mutex>
#include <optional>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/PointCloudPackedUtils.hh>
#include <gz/msgs/image.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/Utility.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <iii_drone_interfaces/msg/camera_frame_ground_truth.hpp>
#include <iii_drone_interfaces/msg/radar_scan_ground_truth.hpp>
#include <iii_drone_interfaces/msg/simulator_drone_state.hpp>
#include <iii_drone_interfaces/msg/static_conductor_geometry.hpp>
#include <sdf/Element.hh>

namespace iii_drone::simulation
{

namespace
{

constexpr double kDefaultMaxPointDist = 18.0;
constexpr double kDefaultViewConeSlope = 0.7;
constexpr double kDefaultPlaneHalfThickness = 0.08;
constexpr double kDefaultSigmaAlong = 0.12;
constexpr double kDefaultSigmaCross = 0.03;
constexpr double kDefaultSigmaNormal = 0.02;
constexpr double kDefaultDropoutProbability = 0.02;
constexpr double kDefaultCorridorMargin = 0.0;
constexpr double kDefaultDopplerResolutionMps = 0.78;
constexpr double kDefaultVelocityNoiseStdMps = 0.08;
constexpr double kDefaultNoiseFloorDb = 40.0;
constexpr double kDefaultNoiseStdDb = 1.5;
constexpr double kDefaultSnrAtOneMeterDb = 30.0;
constexpr double kDefaultSnrRangeFalloffDb = 10.0;
constexpr double kDefaultSnrStdDb = 1.5;
constexpr double kDefaultMinSnrDb = 15.0;
constexpr double kDefaultMaxSnrDb = 30.0;

builtin_interfaces::msg::Time ToRosTime(const gz::msgs::Time & stamp)
{
  builtin_interfaces::msg::Time result;
  result.sec = static_cast<std::int32_t>(stamp.sec());
  result.nanosec = static_cast<std::uint32_t>(stamp.nsec());
  return result;
}

builtin_interfaces::msg::Time ToRosTime(
    const std::chrono::steady_clock::duration & sim_time)
{
  const auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(sim_time).count();
  builtin_interfaces::msg::Time result;
  result.sec = static_cast<std::int32_t>(nanoseconds / 1000000000LL);
  result.nanosec = static_cast<std::uint32_t>(nanoseconds % 1000000000LL);
  return result;
}

geometry_msgs::msg::Point ToRosPoint(const gz::math::Vector3d & point)
{
  geometry_msgs::msg::Point result;
  result.x = point.X();
  result.y = point.Y();
  result.z = point.Z();
  return result;
}

gz::math::Vector3d ToSensorFrame(
    const gz::math::Pose3d & sensor_world_pose,
    const gz::math::Vector3d & world_point)
{
  return sensor_world_pose.Rot().RotateVectorReverse(
      world_point - sensor_world_pose.Pos());
}

gz::math::Vector3d OrthonormalUnit(
    const gz::math::Vector3d & vector,
    const gz::math::Vector3d & reference)
{
  gz::math::Vector3d orthogonal = vector.Cross(reference);
  if (orthogonal.Length() < 1e-9)
  {
    orthogonal = vector.Cross(gz::math::Vector3d::UnitX);
  }
  if (orthogonal.Length() < 1e-9)
  {
    orthogonal = vector.Cross(gz::math::Vector3d::UnitY);
  }
  orthogonal.Normalize();
  return orthogonal;
}

}  // namespace

class MmwaveConductorSensorPlugin :
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPostUpdate
{
  public: void Configure(
      const gz::sim::Entity & entity,
      const std::shared_ptr<const sdf::Element> & sdf,
      gz::sim::EntityComponentManager & ecm,
      gz::sim::EventManager & /*event_mgr*/) override;

  public: void PostUpdate(
      const gz::sim::UpdateInfo & info,
      const gz::sim::EntityComponentManager & ecm) override;

  private: struct Conductor
  {
    std::string id;
    std::vector<gz::math::Vector3d> samples_world;
  };

  private: struct DetectionCandidate
  {
    gz::math::Vector3d point_sensor;
    gz::math::Vector3d tangent_sensor;
    double range{};
    double max_backward_span{};
    double max_forward_span{};
  };

  private: struct DetectedPoint
  {
    gz::math::Vector3d point_sensor;
    gz::math::Vector3d ideal_generating_point_sensor;
    gz::math::Vector3d ideal_generating_point_world;
    gz::math::Vector3d nearest_physical_point_sensor;
    gz::math::Vector3d nearest_physical_point_world;
    std::string physical_conductor_id;
    std::uint16_t conductor_label{};
    std::uint8_t source_class{
      iii_drone_interfaces::msg::RadarPointSource::VALID_PHYSICAL_CONDUCTOR};
    float velocity_mps{};
    float snr_db{};
    float noise_db{};
  };

  private: bool LoadConductors(const std::string & asset_uri);
  private: bool ResolveLink(const gz::sim::EntityComponentManager & ecm);
  private: void UpdateCorridorBounds();
  private: bool IsWithinConductorSpan(
      const gz::math::Vector3d & world_point) const;
  private: std::optional<DetectionCandidate> ProjectClosestPointOnSegment(
      const gz::math::Pose3d & sensor_world_pose,
      const gz::math::Vector3d & start_world,
      const gz::math::Vector3d & end_world) const;
  private: bool IsInFov(const gz::math::Vector3d & point_sensor) const;
  private: gz::math::Vector3d ApplyMeasurementNoise(
      const DetectionCandidate & candidate);
  private: DetectedPoint AddRadarSideInformation(
      const gz::math::Vector3d & point_sensor,
      const gz::math::Vector3d & sensor_velocity_sensor);
  private: void PublishPointCloud(
      const std::chrono::steady_clock::duration & sim_time,
      const std::vector<DetectedPoint> & points);
  private: void PublishDroneState(
      const std::chrono::steady_clock::duration & sim_time,
      const gz::sim::EntityComponentManager & ecm,
      const gz::math::Pose3d & link_world_pose);
  private: void PublishStaticGeometry(const builtin_interfaces::msg::Time & stamp);
  private: void PublishCameraGroundTruth(
      const gz::msgs::Time & stamp,
      const gz::math::Pose3d & link_world_pose);
  private: void OnCameraImage(const gz::msgs::Image & image);

  private: gz::sim::Entity model_entity_{gz::sim::kNullEntity};
  private: gz::sim::Entity link_entity_{gz::sim::kNullEntity};
  private: gz::sim::Link link_;
  private: std::string link_name_{"base_link"};
  private: std::string model_name_{"d4s_dc_drone"};
  private: std::string topic_{"/sensor/mmwave/points"};
  private: std::string full_topic_{"/sensor/mmwave/points_full"};
  private: std::string label_topic_{"/simulation/ground_truth/mmwave/conductor_labels"};
  private: std::string camera_mask_topic_{
      "/simulation/ground_truth/cable_camera/conductor_instance_mask"};
  private: std::string conductor_id_map_topic_{
      "/simulation/ground_truth/conductor_id_map"};
  private: std::string camera_image_topic_{"/sensor/cable_camera/image_raw"};
  private: std::string frame_id_{"mmwave"};
  private: std::string conductor_asset_uri_{
      "model://hcaa_pylon_setup/conductors.yaml"};
  private: gz::math::Pose3d sensor_pose_{
      0.0, 0.0, 0.1, 3.1415, -1.57079632679, 0.0};
  private: gz::math::Pose3d camera_pose_{
      0.0, -0.215, 0.3, 0.0, -1.571, 0.0};
  private: std::uint32_t camera_width_{640};
  private: std::uint32_t camera_height_{480};
  private: double camera_horizontal_fov_rad_{1.3962634};
  private: double update_rate_hz_{30.0};
  private: double max_point_dist_{kDefaultMaxPointDist};
  private: double view_cone_slope_{kDefaultViewConeSlope};
  private: double plane_half_thickness_m_{kDefaultPlaneHalfThickness};
  private: double sigma_along_m_{kDefaultSigmaAlong};
  private: double sigma_cross_m_{kDefaultSigmaCross};
  private: double sigma_normal_m_{kDefaultSigmaNormal};
  private: double dropout_probability_{kDefaultDropoutProbability};
  private: double corridor_margin_m_{kDefaultCorridorMargin};
  private: double doppler_resolution_mps_{kDefaultDopplerResolutionMps};
  private: double velocity_noise_std_mps_{kDefaultVelocityNoiseStdMps};
  private: double noise_floor_db_{kDefaultNoiseFloorDb};
  private: double noise_std_db_{kDefaultNoiseStdDb};
  private: double snr_at_one_meter_db_{kDefaultSnrAtOneMeterDb};
  private: double snr_range_falloff_db_{kDefaultSnrRangeFalloffDb};
  private: double snr_std_db_{kDefaultSnrStdDb};
  private: double min_snr_db_{kDefaultMinSnrDb};
  private: double max_snr_db_{kDefaultMaxSnrDb};
  private: bool stamp_at_latest_tf_{true};
  private: std::uint64_t radar_scan_sequence_{0};
  private: std::uint64_t camera_frame_sequence_{0};
  private: std::chrono::steady_clock::duration state_publish_period_{
      std::chrono::milliseconds(10)};
  private: std::chrono::steady_clock::duration last_state_publish_time_{
      std::chrono::steady_clock::duration::min()};
  private: std::chrono::steady_clock::duration publish_period_{std::chrono::milliseconds(33)};
  private: std::chrono::steady_clock::duration last_publish_time_{
      std::chrono::steady_clock::duration::min()};
  private: gz::transport::Node transport_node_;
  private: gz::transport::Node::Publisher publisher_;
  private: gz::transport::Node::Publisher full_publisher_;
  private: gz::transport::Node::Publisher label_publisher_;
  private: gz::transport::Node::Publisher conductor_id_map_publisher_;
  private: rclcpp::Node::SharedPtr ros_node_;
  private: rclcpp::Publisher<iii_drone_interfaces::msg::SimulatorDroneState>::SharedPtr
      drone_state_publisher_;
  private: rclcpp::Publisher<iii_drone_interfaces::msg::StaticConductorGeometry>::SharedPtr
      static_geometry_publisher_;
  private: rclcpp::Publisher<iii_drone_interfaces::msg::RadarScanGroundTruth>::SharedPtr
      radar_truth_publisher_;
  private: rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera_mask_ros_publisher_;
  private: rclcpp::Publisher<iii_drone_interfaces::msg::CameraFrameGroundTruth>::SharedPtr
      camera_truth_publisher_;
  private: std::vector<Conductor> conductors_;
  private: std::mutex camera_pose_mutex_;
  private: gz::math::Pose3d latest_link_world_pose_;
  private: bool has_latest_link_world_pose_{false};
  private: gz::math::Vector3d corridor_origin_world_{gz::math::Vector3d::Zero};
  private: gz::math::Vector3d corridor_span_axis_world_{gz::math::Vector3d::UnitX};
  private: double corridor_min_span_{0.0};
  private: double corridor_max_span_{0.0};
  private: bool corridor_bounds_valid_{false};
  private: std::mt19937 random_generator_{std::random_device{}()};
  private: std::normal_distribution<double> unit_normal_{0.0, 1.0};
  private: std::uniform_real_distribution<double> unit_uniform_{0.0, 1.0};
  private: bool configured_{false};
};

void MmwaveConductorSensorPlugin::Configure(
    const gz::sim::Entity & entity,
    const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager & ecm,
    gz::sim::EventManager &)
{
  if (const char * seed_text = std::getenv("III_SIMULATION_SEED"))
  {
    try
    {
      this->random_generator_.seed(static_cast<std::mt19937::result_type>(std::stoul(seed_text)));
    }
    catch (const std::exception & error)
    {
      gzerr << "Invalid III_SIMULATION_SEED='" << seed_text << "': " << error.what() << std::endl;
      return;
    }
  }
  this->model_entity_ = entity;

  if (sdf->HasElement("link_name"))
  {
    this->link_name_ = sdf->Get<std::string>("link_name");
  }
  if (sdf->HasElement("topic"))
  {
    this->topic_ = sdf->Get<std::string>("topic");
  }
  this->full_topic_ = this->topic_ + "_full";
  if (sdf->HasElement("full_topic"))
  {
    this->full_topic_ = sdf->Get<std::string>("full_topic");
  }
  if (sdf->HasElement("label_topic"))
  {
    this->label_topic_ = sdf->Get<std::string>("label_topic");
  }
  if (sdf->HasElement("camera_mask_topic"))
  {
    this->camera_mask_topic_ = sdf->Get<std::string>("camera_mask_topic");
  }
  if (sdf->HasElement("camera_image_topic"))
  {
    this->camera_image_topic_ = sdf->Get<std::string>("camera_image_topic");
  }
  if (sdf->HasElement("conductor_id_map_topic"))
  {
    this->conductor_id_map_topic_ = sdf->Get<std::string>("conductor_id_map_topic");
  }
  if (sdf->HasElement("frame_id"))
  {
    this->frame_id_ = sdf->Get<std::string>("frame_id");
  }
  if (sdf->HasElement("conductor_asset_uri"))
  {
    this->conductor_asset_uri_ = sdf->Get<std::string>("conductor_asset_uri");
  }
  if (sdf->HasElement("sensor_pose"))
  {
    this->sensor_pose_ = sdf->Get<gz::math::Pose3d>("sensor_pose");
  }
  if (sdf->HasElement("camera_pose"))
  {
    this->camera_pose_ = sdf->Get<gz::math::Pose3d>("camera_pose");
  }
  if (sdf->HasElement("update_rate_hz"))
  {
    this->update_rate_hz_ = sdf->Get<double>("update_rate_hz");
  }
  if (sdf->HasElement("max_point_dist"))
  {
    this->max_point_dist_ = sdf->Get<double>("max_point_dist");
  }
  if (sdf->HasElement("view_cone_slope"))
  {
    this->view_cone_slope_ = sdf->Get<double>("view_cone_slope");
  }
  if (sdf->HasElement("plane_half_thickness_m"))
  {
    this->plane_half_thickness_m_ = sdf->Get<double>("plane_half_thickness_m");
  }
  if (sdf->HasElement("sigma_along_m"))
  {
    this->sigma_along_m_ = sdf->Get<double>("sigma_along_m");
  }
  if (sdf->HasElement("sigma_cross_m"))
  {
    this->sigma_cross_m_ = sdf->Get<double>("sigma_cross_m");
  }
  if (sdf->HasElement("sigma_normal_m"))
  {
    this->sigma_normal_m_ = sdf->Get<double>("sigma_normal_m");
  }
  if (sdf->HasElement("dropout_probability"))
  {
    this->dropout_probability_ = sdf->Get<double>("dropout_probability");
  }
  if (sdf->HasElement("corridor_margin_m"))
  {
    this->corridor_margin_m_ = sdf->Get<double>("corridor_margin_m");
  }
  if (sdf->HasElement("doppler_resolution_mps"))
  {
    this->doppler_resolution_mps_ = sdf->Get<double>("doppler_resolution_mps");
  }
  if (sdf->HasElement("velocity_noise_std_mps"))
  {
    this->velocity_noise_std_mps_ = sdf->Get<double>("velocity_noise_std_mps");
  }
  if (sdf->HasElement("noise_floor_db"))
  {
    this->noise_floor_db_ = sdf->Get<double>("noise_floor_db");
  }
  if (sdf->HasElement("noise_std_db"))
  {
    this->noise_std_db_ = sdf->Get<double>("noise_std_db");
  }
  if (sdf->HasElement("snr_at_one_meter_db"))
  {
    this->snr_at_one_meter_db_ = sdf->Get<double>("snr_at_one_meter_db");
  }
  if (sdf->HasElement("snr_range_falloff_db"))
  {
    this->snr_range_falloff_db_ = sdf->Get<double>("snr_range_falloff_db");
  }
  if (sdf->HasElement("snr_std_db"))
  {
    this->snr_std_db_ = sdf->Get<double>("snr_std_db");
  }
  if (sdf->HasElement("min_snr_db"))
  {
    this->min_snr_db_ = sdf->Get<double>("min_snr_db");
  }
  if (sdf->HasElement("max_snr_db"))
  {
    this->max_snr_db_ = sdf->Get<double>("max_snr_db");
  }
  if (sdf->HasElement("stamp_at_latest_tf"))
  {
    this->stamp_at_latest_tf_ = sdf->Get<bool>("stamp_at_latest_tf");
  }

  if (this->update_rate_hz_ > 0.0)
  {
    this->publish_period_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / this->update_rate_hz_));
  }
  else
  {
    this->publish_period_ = std::chrono::steady_clock::duration::zero();
  }

  if (!this->ResolveLink(ecm))
  {
    gzerr << "MmwaveConductorSensorPlugin could not resolve link ["
          << this->link_name_ << "] on model entity [" << entity << "].\n";
    return;
  }

  if (!this->LoadConductors(this->conductor_asset_uri_))
  {
    gzerr << "MmwaveConductorSensorPlugin failed to load conductor asset ["
          << this->conductor_asset_uri_ << "].\n";
    return;
  }

  this->model_name_ = gz::sim::Model(entity).Name(ecm);
  if (!rclcpp::ok())
  {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }
  this->ros_node_ = std::make_shared<rclcpp::Node>("sim_ground_truth_publisher");
  this->drone_state_publisher_ = this->ros_node_->create_publisher<
      iii_drone_interfaces::msg::SimulatorDroneState>(
      "/simulation/ground_truth/drone/state", rclcpp::QoS(1000).reliable());
  this->radar_truth_publisher_ = this->ros_node_->create_publisher<
      iii_drone_interfaces::msg::RadarScanGroundTruth>(
      "/simulation/ground_truth/mmwave/scan", rclcpp::QoS(1000).reliable());
  this->camera_mask_ros_publisher_ = this->ros_node_->create_publisher<sensor_msgs::msg::Image>(
      this->camera_mask_topic_, rclcpp::QoS(100).reliable());
  this->camera_truth_publisher_ = this->ros_node_->create_publisher<
      iii_drone_interfaces::msg::CameraFrameGroundTruth>(
      "/simulation/ground_truth/cable_camera/frame", rclcpp::QoS(100).reliable());
  this->static_geometry_publisher_ = this->ros_node_->create_publisher<
      iii_drone_interfaces::msg::StaticConductorGeometry>(
      "/simulation/ground_truth/conductors/geometry",
      rclcpp::QoS(1).reliable().transient_local());
  this->PublishStaticGeometry(ToRosTime(std::chrono::steady_clock::duration::zero()));

  this->link_.EnableVelocityChecks(ecm);

  this->publisher_ = this->transport_node_.Advertise<gz::msgs::PointCloudPacked>(
      this->topic_);
  this->full_publisher_ = this->transport_node_.Advertise<gz::msgs::PointCloudPacked>(
      this->full_topic_);
  this->label_publisher_ = this->transport_node_.Advertise<gz::msgs::PointCloudPacked>(
      this->label_topic_);
  this->conductor_id_map_publisher_ = this->transport_node_.Advertise<gz::msgs::StringMsg>(
      this->conductor_id_map_topic_);
  if (!this->transport_node_.Subscribe(
      this->camera_image_topic_, &MmwaveConductorSensorPlugin::OnCameraImage, this))
  {
    gzerr << "MmwaveConductorSensorPlugin failed to subscribe to camera topic ["
          << this->camera_image_topic_ << "].\n";
    return;
  }
  gzmsg << "MmwaveConductorSensorPlugin configured for link [" << this->link_name_
        << "] with " << this->conductors_.size() << " conductors on topics ["
        << this->topic_ << "] and [" << this->full_topic_ << "].\n";
  this->configured_ = true;
}

void MmwaveConductorSensorPlugin::PostUpdate(
    const gz::sim::UpdateInfo & info,
    const gz::sim::EntityComponentManager & ecm)
{
  if (!this->configured_ || info.paused)
  {
    return;
  }

  if (!this->ResolveLink(ecm))
  {
    return;
  }

  const auto link_world_pose = this->link_.WorldPose(ecm);
  if (!link_world_pose.has_value())
  {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(this->camera_pose_mutex_);
    this->latest_link_world_pose_ = link_world_pose.value();
    this->has_latest_link_world_pose_ = true;
  }

  if (this->last_state_publish_time_ == std::chrono::steady_clock::duration::min() ||
      info.simTime - this->last_state_publish_time_ >= this->state_publish_period_)
  {
    this->PublishDroneState(info.simTime, ecm, link_world_pose.value());
    this->last_state_publish_time_ =
        this->last_state_publish_time_ == std::chrono::steady_clock::duration::min() ?
        info.simTime : this->last_state_publish_time_ + this->state_publish_period_;
  }

  if (this->publish_period_ > std::chrono::steady_clock::duration::zero() &&
      this->last_publish_time_ != std::chrono::steady_clock::duration::min() &&
      info.simTime - this->last_publish_time_ < this->publish_period_)
  {
    return;
  }

  const gz::math::Pose3d sensor_world_pose = link_world_pose.value() * this->sensor_pose_;

  std::vector<DetectedPoint> points;
  points.reserve(this->conductors_.size());

  const gz::math::Vector3d sensor_velocity_world =
      this->link_.WorldLinearVelocity(ecm, this->sensor_pose_.Pos()).value_or(
      gz::math::Vector3d::Zero);
  const gz::math::Vector3d sensor_velocity_sensor =
      sensor_world_pose.Rot().RotateVectorReverse(sensor_velocity_world);

  if (!this->IsWithinConductorSpan(sensor_world_pose.Pos()))
  {
    this->PublishPointCloud(info.simTime, points);
    this->last_publish_time_ = info.simTime;
    return;
  }

  for (std::size_t conductor_index = 0;
      conductor_index < this->conductors_.size(); ++conductor_index)
  {
    const auto & conductor = this->conductors_[conductor_index];
    std::optional<DetectionCandidate> best_candidate;

    for (std::size_t i = 1; i < conductor.samples_world.size(); ++i)
    {
      auto candidate = this->ProjectClosestPointOnSegment(
          sensor_world_pose,
          conductor.samples_world[i - 1],
          conductor.samples_world[i]);

      if (!candidate.has_value())
      {
        continue;
      }

      if (!best_candidate.has_value() ||
          candidate->range < best_candidate->range)
      {
        best_candidate = std::move(candidate);
      }
    }

    if (!best_candidate.has_value())
    {
      continue;
    }

    if (this->unit_uniform_(this->random_generator_) < this->dropout_probability_)
    {
      continue;
    }

    const gz::math::Vector3d noisy_point_sensor =
        this->ApplyMeasurementNoise(best_candidate.value());

    if (!this->IsInFov(noisy_point_sensor))
    {
      continue;
    }

    auto point = this->AddRadarSideInformation(
        noisy_point_sensor, sensor_velocity_sensor);
    point.ideal_generating_point_sensor = best_candidate->point_sensor;
    point.ideal_generating_point_world =
        sensor_world_pose.Pos() + sensor_world_pose.Rot().RotateVector(best_candidate->point_sensor);
    point.nearest_physical_point_sensor = best_candidate->point_sensor;
    point.nearest_physical_point_world = point.ideal_generating_point_world;
    point.physical_conductor_id = conductor.id;
    point.conductor_label = static_cast<std::uint16_t>(conductor_index + 1u);
    points.push_back(std::move(point));
  }

  this->PublishPointCloud(info.simTime, points);
  this->last_publish_time_ = info.simTime;
}

bool MmwaveConductorSensorPlugin::LoadConductors(const std::string & asset_uri)
{
  const std::string asset_path = gz::common::findFile(asset_uri);
  if (asset_path.empty())
  {
    gzerr << "Unable to resolve conductor asset [" << asset_uri << "].\n";
    return false;
  }

  YAML::Node root = YAML::LoadFile(asset_path);
  const YAML::Node conductors_node = root["conductors"];
  if (!conductors_node || !conductors_node.IsSequence())
  {
    gzerr << "Conductor asset [" << asset_path
          << "] does not contain a valid 'conductors' sequence.\n";
    return false;
  }

  this->conductors_.clear();
  this->conductors_.reserve(conductors_node.size());

  for (const auto & conductor_node : conductors_node)
  {
    const YAML::Node samples_node = conductor_node["samples"];
    if (!samples_node || !samples_node.IsSequence() || samples_node.size() < 2)
    {
      continue;
    }

    Conductor conductor;
    if (const YAML::Node id_node = conductor_node["id"]; id_node && id_node.IsScalar())
    {
      conductor.id = id_node.as<std::string>();
    }

    conductor.samples_world.reserve(samples_node.size());
    for (const auto & sample_node : samples_node)
    {
      if (!sample_node.IsSequence() || sample_node.size() != 3)
      {
        continue;
      }

      conductor.samples_world.emplace_back(
          sample_node[0].as<double>(),
          sample_node[1].as<double>(),
          sample_node[2].as<double>());
    }

    if (conductor.samples_world.size() >= 2)
    {
      this->conductors_.push_back(std::move(conductor));
    }
  }

  this->UpdateCorridorBounds();
  return !this->conductors_.empty();
}

bool MmwaveConductorSensorPlugin::ResolveLink(
    const gz::sim::EntityComponentManager & ecm)
{
  if (this->link_entity_ != gz::sim::kNullEntity && this->link_.Valid(ecm))
  {
    return true;
  }

  gz::sim::Model model(this->model_entity_);
  if (!model.Valid(ecm))
  {
    return false;
  }

  this->link_entity_ = model.LinkByName(ecm, this->link_name_);
  if (this->link_entity_ == gz::sim::kNullEntity)
  {
    this->link_entity_ = model.CanonicalLink(ecm);
    gzwarn << "MmwaveConductorSensorPlugin falling back to canonical link for model ["
           << model.Name(ecm) << "] because link [" << this->link_name_
           << "] was not found.\n";
  }

  if (this->link_entity_ == gz::sim::kNullEntity)
  {
    return false;
  }

  this->link_ = gz::sim::Link(this->link_entity_);
  return this->link_.Valid(ecm);
}

void MmwaveConductorSensorPlugin::UpdateCorridorBounds()
{
  this->corridor_bounds_valid_ = false;
  if (this->conductors_.empty())
  {
    return;
  }

  gz::math::Vector3d start_average{0.0, 0.0, 0.0};
  gz::math::Vector3d end_average{0.0, 0.0, 0.0};
  std::size_t endpoint_count = 0;
  for (const auto & conductor : this->conductors_)
  {
    if (conductor.samples_world.size() < 2)
    {
      continue;
    }
    start_average += conductor.samples_world.front();
    end_average += conductor.samples_world.back();
    ++endpoint_count;
  }

  if (endpoint_count == 0)
  {
    return;
  }

  start_average /= static_cast<double>(endpoint_count);
  end_average /= static_cast<double>(endpoint_count);

  gz::math::Vector3d span_axis = end_average - start_average;
  span_axis.Z() = 0.0;
  if (span_axis.Length() < 1e-6)
  {
    return;
  }
  span_axis.Normalize();

  this->corridor_origin_world_ = start_average;
  this->corridor_span_axis_world_ = span_axis;
  this->corridor_min_span_ = std::numeric_limits<double>::infinity();
  this->corridor_max_span_ = -std::numeric_limits<double>::infinity();

  for (const auto & conductor : this->conductors_)
  {
    for (const auto & sample : conductor.samples_world)
    {
      const gz::math::Vector3d relative = sample - this->corridor_origin_world_;
      const double span = relative.Dot(this->corridor_span_axis_world_);
      this->corridor_min_span_ = std::min(this->corridor_min_span_, span);
      this->corridor_max_span_ = std::max(this->corridor_max_span_, span);
    }
  }

  this->corridor_bounds_valid_ =
      std::isfinite(this->corridor_min_span_) &&
      std::isfinite(this->corridor_max_span_);
}

bool MmwaveConductorSensorPlugin::IsWithinConductorSpan(
    const gz::math::Vector3d & world_point) const
{
  if (!this->corridor_bounds_valid_)
  {
    return true;
  }

  const gz::math::Vector3d relative = world_point - this->corridor_origin_world_;
  const double span = relative.Dot(this->corridor_span_axis_world_);
  const double margin = std::max(0.0, this->corridor_margin_m_);

  return span >= this->corridor_min_span_ - margin &&
      span <= this->corridor_max_span_ + margin;
}

std::optional<MmwaveConductorSensorPlugin::DetectionCandidate>
MmwaveConductorSensorPlugin::ProjectClosestPointOnSegment(
    const gz::math::Pose3d & sensor_world_pose,
    const gz::math::Vector3d & start_world,
    const gz::math::Vector3d & end_world) const
{
  const gz::math::Vector3d start_sensor = ToSensorFrame(sensor_world_pose, start_world);
  const gz::math::Vector3d end_sensor = ToSensorFrame(sensor_world_pose, end_world);
  const gz::math::Vector3d segment_sensor = end_sensor - start_sensor;
  const double segment_length = segment_sensor.Length();

  if (segment_length < 1e-6)
  {
    return std::nullopt;
  }

  double interpolation = 0.0;
  const double squared_length = segment_sensor.SquaredLength();
  if (squared_length > 1e-12)
  {
    interpolation = std::clamp(
        -start_sensor.Dot(segment_sensor) / squared_length,
        0.0,
        1.0);
  }

  const gz::math::Vector3d point_sensor =
      start_sensor + segment_sensor * interpolation;
  if (!this->IsInFov(point_sensor))
  {
    return std::nullopt;
  }

  DetectionCandidate candidate;
  candidate.point_sensor = point_sensor;
  candidate.tangent_sensor = segment_sensor / segment_length;
  candidate.range = point_sensor.Length();
  candidate.max_backward_span = interpolation * segment_length;
  candidate.max_forward_span = (1.0 - interpolation) * segment_length;
  return candidate;
}

bool MmwaveConductorSensorPlugin::IsInFov(
    const gz::math::Vector3d & point_sensor) const
{
  const double range = point_sensor.Length();
  if (range > this->max_point_dist_)
  {
    return false;
  }

  const double yz_distance = std::hypot(point_sensor.Y(), point_sensor.Z());
  return point_sensor.X() > this->view_cone_slope_ * yz_distance;
}

gz::math::Vector3d MmwaveConductorSensorPlugin::ApplyMeasurementNoise(
    const DetectionCandidate & candidate)
{
  const double along_noise = std::clamp(
      this->unit_normal_(this->random_generator_) * this->sigma_along_m_,
      -candidate.max_backward_span,
      candidate.max_forward_span);

  gz::math::Vector3d noisy_point =
      candidate.point_sensor + candidate.tangent_sensor * along_noise;

  const gz::math::Vector3d cross_axis =
      OrthonormalUnit(candidate.tangent_sensor, gz::math::Vector3d::UnitZ);
  const gz::math::Vector3d normal_axis =
      candidate.tangent_sensor.Cross(cross_axis).Normalized();

  noisy_point += cross_axis *
      (this->unit_normal_(this->random_generator_) * this->sigma_cross_m_);
  noisy_point += normal_axis *
      (this->unit_normal_(this->random_generator_) * this->sigma_normal_m_);

  return noisy_point;
}

MmwaveConductorSensorPlugin::DetectedPoint
MmwaveConductorSensorPlugin::AddRadarSideInformation(
    const gz::math::Vector3d & point_sensor,
    const gz::math::Vector3d & sensor_velocity_sensor)
{
  DetectedPoint point;
  point.point_sensor = point_sensor;

  const double range = point_sensor.Length();
  if (range > 1e-6)
  {
    // Conductors are static in the simulation. Their relative range rate is
    // therefore the negative projection of sensor velocity onto the line of
    // sight. Quantisation approximates the supplied 18 m radar profile.
    const double range_rate =
        -sensor_velocity_sensor.Dot(point_sensor / range);
    const double noisy_range_rate = range_rate +
        this->unit_normal_(this->random_generator_) * this->velocity_noise_std_mps_;
    if (this->doppler_resolution_mps_ > 0.0)
    {
      point.velocity_mps = static_cast<float>(
          std::round(noisy_range_rate / this->doppler_resolution_mps_) *
          this->doppler_resolution_mps_);
    }
    else
    {
      point.velocity_mps = static_cast<float>(noisy_range_rate);
    }
  }

  // TI reports CFAR noise and SNR as logarithmic values. Approximately 40 dB
  // noise and 10--30 dB SNR are representative indoor IWR6843 observations.
  // Detected points are clamped at the configured 15 dB CFAR threshold.
  point.noise_db = static_cast<float>(std::max(
      0.0,
      this->noise_floor_db_ +
      this->unit_normal_(this->random_generator_) * this->noise_std_db_));
  const double range_loss_db = this->snr_range_falloff_db_ *
      std::log10(std::max(1.0, range));
  point.snr_db = static_cast<float>(std::clamp(
      this->snr_at_one_meter_db_ - range_loss_db +
      this->unit_normal_(this->random_generator_) * this->snr_std_db_,
      this->min_snr_db_,
      this->max_snr_db_));

  return point;
}

void MmwaveConductorSensorPlugin::PublishPointCloud(
    const std::chrono::steady_clock::duration & sim_time,
    const std::vector<DetectedPoint> & points)
{
  gz::msgs::PointCloudPacked message;
  gz::msgs::InitPointCloudPacked(
      message,
      this->frame_id_,
      false,
      {{"xyz", gz::msgs::PointCloudPacked::Field::FLOAT32}});
  gz::msgs::PointCloudPacked full_message;
  gz::msgs::InitPointCloudPacked(
      full_message,
      this->frame_id_,
      false,
      {{"xyz", gz::msgs::PointCloudPacked::Field::FLOAT32},
       {"velocity", gz::msgs::PointCloudPacked::Field::FLOAT32},
       {"snr", gz::msgs::PointCloudPacked::Field::FLOAT32},
       {"noise", gz::msgs::PointCloudPacked::Field::FLOAT32}});
  gz::msgs::PointCloudPacked label_message;
  gz::msgs::InitPointCloudPacked(
      label_message,
      this->frame_id_,
      false,
      {{"conductor_id", gz::msgs::PointCloudPacked::Field::UINT16}});

  // Keep the legacy visualization cloud's latest-TF behavior, but always stamp
  // the dataset cloud and its point-aligned labels with simulation time. This
  // makes every measurement-label pair unambiguous offline.
  *full_message.mutable_header()->mutable_stamp() = gz::msgs::Convert(sim_time);
  *label_message.mutable_header()->mutable_stamp() = gz::msgs::Convert(sim_time);
  if (!this->stamp_at_latest_tf_)
  {
    *message.mutable_header()->mutable_stamp() = gz::msgs::Convert(sim_time);
  }
  message.set_height(1);
  message.set_width(static_cast<std::uint32_t>(points.size()));
  message.set_is_dense(true);
  message.set_row_step(message.point_step() * message.width());
  message.mutable_data()->resize(points.size() * message.point_step());
  full_message.set_height(1);
  full_message.set_width(static_cast<std::uint32_t>(points.size()));
  full_message.set_is_dense(true);
  full_message.set_row_step(full_message.point_step() * full_message.width());
  full_message.mutable_data()->resize(points.size() * full_message.point_step());
  label_message.set_height(1);
  label_message.set_width(static_cast<std::uint32_t>(points.size()));
  label_message.set_is_dense(true);
  label_message.set_row_step(label_message.point_step() * label_message.width());
  label_message.mutable_data()->resize(points.size() * label_message.point_step());

  if (!points.empty())
  {
    gz::msgs::PointCloudPackedIterator<float> x_it(message, "x");
    gz::msgs::PointCloudPackedIterator<float> y_it(message, "y");
    gz::msgs::PointCloudPackedIterator<float> z_it(message, "z");
    gz::msgs::PointCloudPackedIterator<float> full_x_it(full_message, "x");
    gz::msgs::PointCloudPackedIterator<float> full_y_it(full_message, "y");
    gz::msgs::PointCloudPackedIterator<float> full_z_it(full_message, "z");
    gz::msgs::PointCloudPackedIterator<float> velocity_it(full_message, "velocity");
    gz::msgs::PointCloudPackedIterator<float> snr_it(full_message, "snr");
    gz::msgs::PointCloudPackedIterator<float> noise_it(full_message, "noise");
    gz::msgs::PointCloudPackedIterator<std::uint16_t> conductor_id_it(
        label_message, "conductor_id");

    for (const auto & point : points)
    {
      const float x = static_cast<float>(point.point_sensor.X());
      const float y = static_cast<float>(point.point_sensor.Y());
      const float z = static_cast<float>(point.point_sensor.Z());
      *x_it = x;
      *y_it = y;
      *z_it = z;
      *full_x_it = x;
      *full_y_it = y;
      *full_z_it = z;
      *velocity_it = point.velocity_mps;
      *snr_it = point.snr_db;
      *noise_it = point.noise_db;
      *conductor_id_it = point.conductor_label;
      ++x_it;
      ++y_it;
      ++z_it;
      ++full_x_it;
      ++full_y_it;
      ++full_z_it;
      ++velocity_it;
      ++snr_it;
      ++noise_it;
      ++conductor_id_it;
    }
  }

  this->publisher_.Publish(message);
  this->full_publisher_.Publish(full_message);
  this->label_publisher_.Publish(label_message);

  iii_drone_interfaces::msg::RadarScanGroundTruth truth;
  truth.header.stamp = ToRosTime(sim_time);
  truth.header.frame_id = this->frame_id_;
  truth.source_topic = this->full_topic_;
  truth.scan_sequence = this->radar_scan_sequence_++;
  truth.points.reserve(points.size());
  for (std::size_t index = 0; index < points.size(); ++index)
  {
    const auto & point = points[index];
    iii_drone_interfaces::msg::RadarPointSource source;
    source.source_point_index = static_cast<std::uint32_t>(index);
    source.source_class = point.source_class;
    source.physical_conductor_id = point.physical_conductor_id;
    source.ideal_generating_point_world = ToRosPoint(point.ideal_generating_point_world);
    source.ideal_generating_point_sensor = ToRosPoint(point.ideal_generating_point_sensor);
    source.nearest_physical_point_world = ToRosPoint(point.nearest_physical_point_world);
    source.nearest_physical_point_sensor = ToRosPoint(point.nearest_physical_point_sensor);
    source.generating_point_equals_nearest_point = true;
    truth.points.push_back(std::move(source));
  }
  this->radar_truth_publisher_->publish(truth);
}

void MmwaveConductorSensorPlugin::PublishDroneState(
    const std::chrono::steady_clock::duration & sim_time,
    const gz::sim::EntityComponentManager & ecm,
    const gz::math::Pose3d & link_world_pose)
{
  iii_drone_interfaces::msg::SimulatorDroneState state;
  state.header.stamp = ToRosTime(sim_time);
  state.header.frame_id = "world";
  state.source_model_name = this->model_name_;
  state.source_link_name = this->link_.Name(ecm).value_or(this->link_name_);
  state.pose_world.position = ToRosPoint(link_world_pose.Pos());
  state.pose_world.orientation.x = link_world_pose.Rot().X();
  state.pose_world.orientation.y = link_world_pose.Rot().Y();
  state.pose_world.orientation.z = link_world_pose.Rot().Z();
  state.pose_world.orientation.w = link_world_pose.Rot().W();
  const auto linear = this->link_.WorldLinearVelocity(ecm).value_or(gz::math::Vector3d::Zero);
  const auto angular = this->link_.WorldAngularVelocity(ecm).value_or(gz::math::Vector3d::Zero);
  state.twist_world.linear.x = linear.X();
  state.twist_world.linear.y = linear.Y();
  state.twist_world.linear.z = linear.Z();
  state.twist_world.angular.x = angular.X();
  state.twist_world.angular.y = angular.Y();
  state.twist_world.angular.z = angular.Z();
  this->drone_state_publisher_->publish(state);
}

void MmwaveConductorSensorPlugin::PublishStaticGeometry(
    const builtin_interfaces::msg::Time & stamp)
{
  iii_drone_interfaces::msg::StaticConductorGeometry geometry;
  geometry.header.stamp = stamp;
  geometry.header.frame_id = "world";
  geometry.geometry_revision = "hcaa_pylon_setup/conductors.yaml:v1";
  geometry.conductors.reserve(this->conductors_.size());
  for (const auto & conductor : this->conductors_)
  {
    iii_drone_interfaces::msg::ConductorGeometry item;
    item.physical_id = conductor.id;
    item.physical_radius_m = 0.018;
    item.centerline_samples_world.reserve(conductor.samples_world.size());
    for (const auto & point : conductor.samples_world)
    {
      item.centerline_samples_world.push_back(ToRosPoint(point));
    }
    geometry.conductors.push_back(std::move(item));
  }
  this->static_geometry_publisher_->publish(geometry);
}

void MmwaveConductorSensorPlugin::PublishCameraGroundTruth(
    const gz::msgs::Time & stamp,
    const gz::math::Pose3d & link_world_pose)
{
  const gz::math::Pose3d camera_world_pose = link_world_pose * this->camera_pose_;
  const double focal_length =
      0.5 * static_cast<double>(this->camera_width_) /
      std::tan(0.5 * this->camera_horizontal_fov_rad_);
  const double center_x = 0.5 * (static_cast<double>(this->camera_width_) - 1.0);
  const double center_y = 0.5 * (static_cast<double>(this->camera_height_) - 1.0);
  const std::size_t pixel_count =
      static_cast<std::size_t>(this->camera_width_) * this->camera_height_;
  std::vector<std::uint16_t> labels(pixel_count, 0u);
  std::vector<double> depths(pixel_count, std::numeric_limits<double>::infinity());
  std::vector<bool> projected_in_fov(this->conductors_.size(), false);

  for (std::size_t conductor_index = 0;
      conductor_index < this->conductors_.size(); ++conductor_index)
  {
    const auto & samples = this->conductors_[conductor_index].samples_world;
    const auto label = static_cast<std::uint16_t>(conductor_index + 1u);
    for (std::size_t segment_index = 1; segment_index < samples.size(); ++segment_index)
    {
      const gz::math::Vector3d segment = samples[segment_index] - samples[segment_index - 1];
      const int steps = std::max(1, static_cast<int>(std::ceil(segment.Length() / 0.015)));
      for (int step = 0; step <= steps; ++step)
      {
        const double ratio = static_cast<double>(step) / static_cast<double>(steps);
        const auto point_world = samples[segment_index - 1] + segment * ratio;
        const auto point_camera = ToSensorFrame(camera_world_pose, point_world);
        if (point_camera.X() <= 0.02)
        {
          continue;
        }
        const int pixel_x = static_cast<int>(std::lround(
            center_x - focal_length * point_camera.Y() / point_camera.X()));
        const int pixel_y = static_cast<int>(std::lround(
            center_y - focal_length * point_camera.Z() / point_camera.X()));
        const int radius = std::max(
            1, static_cast<int>(std::ceil(focal_length * 0.018 / point_camera.X())));
        if (pixel_x + radius >= 0 && pixel_x - radius < static_cast<int>(this->camera_width_) &&
            pixel_y + radius >= 0 && pixel_y - radius < static_cast<int>(this->camera_height_))
        {
          projected_in_fov[conductor_index] = true;
        }
        for (int y = pixel_y - radius; y <= pixel_y + radius; ++y)
        {
          if (y < 0 || y >= static_cast<int>(this->camera_height_))
          {
            continue;
          }
          for (int x = pixel_x - radius; x <= pixel_x + radius; ++x)
          {
            if (x < 0 || x >= static_cast<int>(this->camera_width_) ||
                (x - pixel_x) * (x - pixel_x) + (y - pixel_y) * (y - pixel_y) > radius * radius)
            {
              continue;
            }
            const std::size_t offset =
                static_cast<std::size_t>(y) * this->camera_width_ + static_cast<std::size_t>(x);
            if (point_camera.X() < depths[offset])
            {
              depths[offset] = point_camera.X();
              labels[offset] = label;
            }
          }
        }
      }
    }
  }

  gz::msgs::Image mask;
  *mask.mutable_header()->mutable_stamp() = stamp;
  auto * frame = mask.mutable_header()->add_data();
  frame->set_key("frame_id");
  frame->add_value("cable_camera");
  mask.set_width(this->camera_width_);
  mask.set_height(this->camera_height_);
  mask.set_step(this->camera_width_ * sizeof(std::uint16_t));
  mask.set_pixel_format_type(gz::msgs::PixelFormatType::L_INT16);
  mask.set_data(labels.data(), labels.size() * sizeof(std::uint16_t));

  sensor_msgs::msg::Image ros_mask;
  ros_mask.header.stamp = ToRosTime(stamp);
  ros_mask.header.frame_id = "cable_camera";
  ros_mask.width = this->camera_width_;
  ros_mask.height = this->camera_height_;
  ros_mask.encoding = "mono16";
  ros_mask.is_bigendian = false;
  ros_mask.step = this->camera_width_ * sizeof(std::uint16_t);
  ros_mask.data.resize(labels.size() * sizeof(std::uint16_t));
  std::memcpy(ros_mask.data.data(), labels.data(), ros_mask.data.size());
  this->camera_mask_ros_publisher_->publish(ros_mask);

  iii_drone_interfaces::msg::CameraFrameGroundTruth frame_truth;
  frame_truth.header = ros_mask.header;
  frame_truth.source_image_topic = this->camera_image_topic_;
  frame_truth.instance_mask_topic = this->camera_mask_topic_;
  frame_truth.frame_sequence = this->camera_frame_sequence_++;
  frame_truth.image_width = this->camera_width_;
  frame_truth.image_height = this->camera_height_;
  frame_truth.conductors.reserve(this->conductors_.size());
  for (std::size_t conductor_index = 0;
      conductor_index < this->conductors_.size(); ++conductor_index)
  {
    const auto label = static_cast<std::uint16_t>(conductor_index + 1u);
    iii_drone_interfaces::msg::CameraConductorVisibility visibility;
    visibility.physical_conductor_id = this->conductors_[conductor_index].id;
    visibility.mask_value = label;
    std::uint32_t min_u = this->camera_width_;
    std::uint32_t min_v = this->camera_height_;
    std::uint32_t max_u = 0;
    std::uint32_t max_v = 0;
    for (std::uint32_t v = 0; v < this->camera_height_; ++v)
    {
      for (std::uint32_t u = 0; u < this->camera_width_; ++u)
      {
        if (labels[static_cast<std::size_t>(v) * this->camera_width_ + u] == label)
        {
          ++visibility.visible_pixel_count;
          min_u = std::min(min_u, u);
          min_v = std::min(min_v, v);
          max_u = std::max(max_u, u);
          max_v = std::max(max_v, v);
        }
      }
    }
    visibility.has_visible_bounding_box = visibility.visible_pixel_count > 0;
    if (visibility.has_visible_bounding_box)
    {
      visibility.visibility_state =
          iii_drone_interfaces::msg::CameraConductorVisibility::VISIBLE;
      visibility.min_u = min_u;
      visibility.min_v = min_v;
      visibility.max_u = max_u;
      visibility.max_v = max_v;
    }
    else
    {
      visibility.visibility_state = projected_in_fov[conductor_index] ?
          iii_drone_interfaces::msg::CameraConductorVisibility::NO_VISIBLE_PIXELS :
          iii_drone_interfaces::msg::CameraConductorVisibility::OUTSIDE_FOV;
    }
    frame_truth.conductors.push_back(std::move(visibility));
  }
  this->camera_truth_publisher_->publish(frame_truth);
  this->PublishStaticGeometry(ros_mask.header.stamp);

  std::string mapping{"{\"0\":\"background\""};
  for (std::size_t index = 0; index < this->conductors_.size(); ++index)
  {
    mapping += ",\"" + std::to_string(index + 1u) + "\":\"" +
      this->conductors_[index].id + "\"";
  }
  mapping += "}";
  gz::msgs::StringMsg id_map;
  id_map.set_data(mapping);
  this->conductor_id_map_publisher_.Publish(id_map);
}

void MmwaveConductorSensorPlugin::OnCameraImage(const gz::msgs::Image & image)
{
  gz::math::Pose3d link_world_pose;
  {
    std::lock_guard<std::mutex> lock(this->camera_pose_mutex_);
    if (!this->has_latest_link_world_pose_)
    {
      return;
    }
    link_world_pose = this->latest_link_world_pose_;
  }
  this->PublishCameraGroundTruth(image.header().stamp(), link_world_pose);
}

}  // namespace iii_drone::simulation

GZ_ADD_PLUGIN(
    iii_drone::simulation::MmwaveConductorSensorPlugin,
    gz::sim::System,
    gz::sim::ISystemConfigure,
    gz::sim::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(
    iii_drone::simulation::MmwaveConductorSensorPlugin,
    "iii_drone::simulation::MmwaveConductorSensorPlugin")
