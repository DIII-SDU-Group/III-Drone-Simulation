#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <map>
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
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/header.hpp>
#include <iii_drone_interfaces/msg/camera_frame_ground_truth.hpp>
#include <iii_drone_interfaces/msg/pylon_camera_frame_ground_truth.hpp>
#include <iii_drone_interfaces/msg/pylon_exact_mask_frame_ground_truth.hpp>
#include <iii_drone_interfaces/msg/radar_scan_ground_truth.hpp>
#include <iii_drone_interfaces/msg/radar_scan_truth_v2.hpp>
#include <iii_drone_interfaces/msg/simulator_drone_state.hpp>
#include <iii_drone_interfaces/msg/static_conductor_geometry.hpp>
#include <sdf/Element.hh>

#include "iii_drone_simulation/mmwave_fov_geometry.hpp"
#include "iii_drone_simulation/aop_radar_config.hpp"
#include "iii_drone_simulation/aop_radar_model.hpp"
#include "iii_drone_simulation/aop_radar_accel.hpp"

namespace iii_drone::simulation
{

namespace
{

constexpr double kDefaultMaxPointDist = 18.0;
constexpr double kDefaultMinPointDist = 0.25;
constexpr double kDefaultViewConeSlope = 0.7;
// Half-angle of the FINITE_RECTANGULAR FOV used for powerline SLAM development.
constexpr double kDefaultFiniteFovHalfAngleRad = 0.6107259643892086;
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
constexpr std::size_t kCameraRegistrationWindowFrames = 64u;

builtin_interfaces::msg::Time ToRosTime(const gz::msgs::Time & stamp)
{
  builtin_interfaces::msg::Time result;
  result.sec = static_cast<std::int32_t>(stamp.sec());
  result.nanosec = static_cast<std::uint32_t>(stamp.nsec());
  return result;
}

std::int64_t StampNanoseconds(const gz::msgs::Time & stamp)
{
  return static_cast<std::int64_t>(stamp.sec()) * 1000000000LL +
         static_cast<std::int64_t>(stamp.nsec());
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
    double segment_parameter{};
    double conductor_parameter{};
    std::uint16_t active_support_boundaries{};
    double line_of_sight_tangent_angle_rad{};
  };

  private: struct Pylon
  {
    std::string id;
    gz::math::Vector3d aabb_min_world;
    gz::math::Vector3d aabb_max_world;
  };

  private: struct PendingPylonFrame
  {
    gz::msgs::Image image;
    gz::math::Pose3d link_world_pose;
  };

  private: struct DetectedPoint
  {
    gz::math::Vector3d point_sensor;
    gz::math::Vector3d ideal_generating_point_sensor;
    gz::math::Vector3d ideal_generating_point_world;
    gz::math::Vector3d nearest_physical_point_sensor;
    gz::math::Vector3d nearest_physical_point_world;
    std::string physical_conductor_id;
    std::string physical_pylon_id;
    std::uint16_t conductor_label{};
    std::uint8_t source_class{
      iii_drone_interfaces::msg::RadarPointSource::VALID_PHYSICAL_CONDUCTOR};
    float velocity_mps{};
    float snr_db{};
    float noise_db{};
    double conductor_parameter{};
    std::uint16_t active_support_boundaries{};
    std::uint8_t generating_geometry_class{};
    double unconstrained_nearest_distance_m{};
    double line_of_sight_tangent_angle_rad{};
    bool generating_point_equals_nearest_point{};
  };

  private: bool LoadConductors(const std::string & asset_uri);
  private: bool LoadPylons(const std::string & asset_uri);
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
      const gz::math::Pose3d & link_world_pose,
      std::uint64_t frame_sequence);
  private: void PublishPylonSemanticImage(
      const gz::msgs::Image & image,
      const gz::math::Pose3d & link_world_pose,
      std::uint64_t frame_sequence);
  private: void OnCameraImage(const gz::msgs::Image & image);

  private: void PublishRosCameraImage(const gz::msgs::Image & image);
  // r21 simulator-v2 (AOP_FAST_POINT) path; LEGACY_GEOMETRIC keeps the original code unchanged.
  private: void AopUpdate(
      const gz::sim::UpdateInfo & info,
      const gz::sim::EntityComponentManager & ecm,
      const gz::math::Pose3d & link_world_pose);
  private: void PublishAopScan(
      const std::chrono::steady_clock::duration & sim_time,
      const std::vector<aop_radar::Detection> & detections,
      const aop_radar::ScanDiagnostics & diag,
      const gz::math::Pose3d & sensor_world_pose,
      const gz::math::Vector3d & sensor_velocity_world,
      std::uint64_t frame_index, double jitter_s, double overlap);
  private: void OnPylonSemanticImage(const gz::msgs::Image & image);

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
  private: std::string camera_info_topic_{"/sensor/cable_camera/camera_info"};
  private: std::string pylon_semantic_topic_{
      "/simulation/ground_truth/cable_camera/pylon_semantic_raw/labels_map"};
  private: std::string pylon_mask_topic_{
      "/simulation/ground_truth/cable_camera/pylon_instance_mask"};
  private: std::string frame_id_{"mmwave"};
  private: std::string conductor_asset_uri_{
      "model://hcaa_pylon_setup/conductors.yaml"};
  // Empty disables the evaluator-only pylon map, pylon returns and pylon truth.
  private: std::string pylon_asset_uri_;
  private: gz::math::Pose3d sensor_pose_{
      0.0, 0.0, 0.1, 3.1415, -1.57079632679, 0.0};
  private: gz::math::Pose3d camera_pose_{
      0.0, -0.215, 0.3, 0.0, -1.571, 0.0};
  private: std::uint32_t camera_width_{640};
  private: std::uint32_t camera_height_{480};
  private: double camera_horizontal_fov_rad_{1.3962634};
  private: double update_rate_hz_{30.0};
  private: double max_point_dist_{kDefaultMaxPointDist};
  private: double min_point_dist_{kDefaultMinPointDist};
  private: double view_cone_slope_{kDefaultViewConeSlope};
  private: double azimuth_half_angle_rad_{kDefaultFiniteFovHalfAngleRad};
  private: double elevation_half_angle_rad_{kDefaultFiniteFovHalfAngleRad};
  // VIEW_CONE: visible when x > view_cone_slope * hypot(y, z) and range <= max.
  // FINITE_RECTANGULAR: min/max range and azimuth/elevation half-angles.
  private: std::string fov_model_{"VIEW_CONE"};
  private: bool finite_fov_{false};
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
  private: bool pylon_returns_enabled_{false};
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
  private: rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera_image_ros_publisher_;
  private: rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pylon_mask_ros_publisher_;
  private: rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr
      camera_info_publisher_;
  private: rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      full_cloud_ros_publisher_;
  private: rclcpp::Publisher<iii_drone_interfaces::msg::CameraFrameGroundTruth>::SharedPtr
      camera_truth_publisher_;
  private: rclcpp::Publisher<iii_drone_interfaces::msg::PylonCameraFrameGroundTruth>::SharedPtr
      pylon_camera_truth_publisher_;
  private: rclcpp::Publisher<iii_drone_interfaces::msg::PylonExactMaskFrameGroundTruth>::SharedPtr
      pylon_exact_mask_truth_publisher_;
  private: std::vector<Conductor> conductors_;
  private: std::vector<Pylon> pylons_;
  private: std::string pylon_geometry_revision_;
  private: std::mutex camera_pose_mutex_;
  private: std::mutex camera_registration_mutex_;
  private: std::map<std::int64_t, std::uint64_t> camera_frame_identities_;
  private: std::map<std::int64_t, PendingPylonFrame> pending_pylon_frames_;
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
  // r21 multi-instance / simulator-v2 configuration
  private: std::string radar_model_{"LEGACY_GEOMETRIC"};
  private: std::string aop_backend_{"cpu"};
  private: std::string radar_instance_{"mmwave"};
  private: std::string aop_config_path_;
  private: std::uint64_t radar_seed_{0};
  private: std::uint64_t simulation_seed_{0};
  private: bool publish_camera_{true};
  private: bool publish_drone_state_{true};
  private: bool publish_static_geometry_{true};
  // Direct ROS copies of the camera image and full radar cloud. Off by default:
  // sim_assets.launch.py bridges those topics from Gazebo.
  private: bool publish_ros_sensor_streams_{false};
  private: bool publish_camera_info_{false};
  private: std::string truth_v2_topic_;
  private: double schedule_offset_ms_{0.0};
  private: double schedule_jitter_sigma_us_{0.0};
  private: double peer_offset_ms_{-1.0};
  private: double peer_active_ms_{0.0};
  private: bool aop_mode_{false};
  private: aop_radar::ModelConfig aop_config_;
  private: aop_radar::AopRadarModel aop_model_;
  private: std::uint64_t aop_next_frame_{0};
  private: std::uint64_t aop_scan_sequence_{0};
  private: rclcpp::Publisher<iii_drone_interfaces::msg::RadarScanTruthV2>::SharedPtr truth_v2_publisher_;
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
      this->simulation_seed_ = static_cast<std::uint64_t>(std::stoull(seed_text));
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
  if (sdf->HasElement("camera_info_topic"))
  {
    this->camera_info_topic_ = sdf->Get<std::string>("camera_info_topic");
  }
  if (sdf->HasElement("pylon_semantic_topic"))
  {
    this->pylon_semantic_topic_ = sdf->Get<std::string>("pylon_semantic_topic");
  }
  if (sdf->HasElement("pylon_mask_topic"))
  {
    this->pylon_mask_topic_ = sdf->Get<std::string>("pylon_mask_topic");
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
  if (sdf->HasElement("pylon_asset_uri"))
  {
    this->pylon_asset_uri_ = sdf->Get<std::string>("pylon_asset_uri");
  }
  if (sdf->HasElement("pylon_returns_enabled"))
  {
    this->pylon_returns_enabled_ = sdf->Get<bool>("pylon_returns_enabled");
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
  if (sdf->HasElement("min_point_dist"))
  {
    this->min_point_dist_ = sdf->Get<double>("min_point_dist");
  }
  if (sdf->HasElement("view_cone_slope"))
  {
    this->view_cone_slope_ = sdf->Get<double>("view_cone_slope");
  }
  if (sdf->HasElement("azimuth_half_angle_rad"))
  {
    this->azimuth_half_angle_rad_ = sdf->Get<double>("azimuth_half_angle_rad");
  }
  if (sdf->HasElement("elevation_half_angle_rad"))
  {
    this->elevation_half_angle_rad_ = sdf->Get<double>("elevation_half_angle_rad");
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
  // r21 / WO-2026-09-29-001 multi-instance + simulator-v2 parameters (all optional; defaults = legacy behaviour).
  if (sdf->HasElement("radar_model")) {this->radar_model_ = sdf->Get<std::string>("radar_model");}
  if (sdf->HasElement("radar_instance")) {this->radar_instance_ = sdf->Get<std::string>("radar_instance");}
  if (sdf->HasElement("aop_config")) {this->aop_config_path_ = sdf->Get<std::string>("aop_config");}
  if (sdf->HasElement("radar_seed")) {this->radar_seed_ = static_cast<std::uint64_t>(sdf->Get<int>("radar_seed"));}
  if (sdf->HasElement("publish_camera")) {this->publish_camera_ = sdf->Get<bool>("publish_camera");}
  if (sdf->HasElement("publish_drone_state")) {this->publish_drone_state_ = sdf->Get<bool>("publish_drone_state");}
  if (sdf->HasElement("publish_static_geometry")) {this->publish_static_geometry_ = sdf->Get<bool>("publish_static_geometry");}
  if (sdf->HasElement("publish_ros_sensor_streams")) {this->publish_ros_sensor_streams_ = sdf->Get<bool>("publish_ros_sensor_streams");}
  if (sdf->HasElement("publish_camera_info")) {this->publish_camera_info_ = sdf->Get<bool>("publish_camera_info");}
  if (sdf->HasElement("fov_model")) {this->fov_model_ = sdf->Get<std::string>("fov_model");}
  this->truth_v2_topic_ = "/simulation/ground_truth/" + this->radar_instance_ + "/scan_v2";
  if (sdf->HasElement("truth_v2_topic")) {this->truth_v2_topic_ = sdf->Get<std::string>("truth_v2_topic");}
  if (sdf->HasElement("schedule_offset_ms")) {this->schedule_offset_ms_ = sdf->Get<double>("schedule_offset_ms");}
  if (sdf->HasElement("schedule_jitter_sigma_us")) {this->schedule_jitter_sigma_us_ = sdf->Get<double>("schedule_jitter_sigma_us");}
  if (sdf->HasElement("peer_offset_ms")) {this->peer_offset_ms_ = sdf->Get<double>("peer_offset_ms");}
  if (sdf->HasElement("peer_active_ms")) {this->peer_active_ms_ = sdf->Get<double>("peer_active_ms");}
  if (this->radar_model_ == "AOP_FAST_POINT")
  {
    try
    {
      this->aop_config_ = aop_radar::load_model_config_file(gz::common::findFile(this->aop_config_path_));
    }
    catch (const std::exception & error)
    {
      gzerr << "MmwaveConductorSensorPlugin [" << this->radar_instance_ << "] failed to load AOP config ["
            << this->aop_config_path_ << "]: " << error.what() << "\n";
      return;
    }
    this->aop_mode_ = true;
  }
  else if (this->radar_model_ != "LEGACY_GEOMETRIC")
  {
    gzerr << "MmwaveConductorSensorPlugin: unknown radar_model [" << this->radar_model_ << "]\n";
    return;
  }

  if (this->fov_model_ == "FINITE_RECTANGULAR")
  {
    const mmwave_fov::SensorFov configured_fov{
        this->min_point_dist_, this->max_point_dist_, this->azimuth_half_angle_rad_,
        this->elevation_half_angle_rad_};
    if (!mmwave_fov::IsFiniteFov(configured_fov))
    {
      gzerr << "MmwaveConductorSensorPlugin has an invalid finite FOV/range configuration.\n";
      return;
    }
    this->finite_fov_ = true;
  }
  else if (this->fov_model_ != "VIEW_CONE")
  {
    gzerr << "MmwaveConductorSensorPlugin: unknown fov_model [" << this->fov_model_ << "]\n";
    return;
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
  if (!this->LoadPylons(this->pylon_asset_uri_))
  {
    gzerr << "MmwaveConductorSensorPlugin failed to load pylon evaluator asset ["
          << this->pylon_asset_uri_ << "].\n";
    return;
  }

  this->model_name_ = gz::sim::Model(entity).Name(ecm);
  if (!rclcpp::ok())
  {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }
  this->ros_node_ = std::make_shared<rclcpp::Node>(
      this->radar_instance_ == "mmwave" ? std::string("sim_ground_truth_publisher") :
      "sim_ground_truth_publisher_" + this->radar_instance_);
  // r21: a secondary radar instance advertises only its own radar topics (no camera/state/geometry duplicates).
  if (this->publish_drone_state_)
  {
    this->drone_state_publisher_ = this->ros_node_->create_publisher<
        iii_drone_interfaces::msg::SimulatorDroneState>(
        "/simulation/ground_truth/drone/state", rclcpp::QoS(1000).reliable());
  }
  if (!this->aop_mode_)
  {
    this->radar_truth_publisher_ = this->ros_node_->create_publisher<
        iii_drone_interfaces::msg::RadarScanGroundTruth>(
        "/simulation/ground_truth/mmwave/scan", rclcpp::QoS(1000).reliable());
  }
  // Optional direct ROS copies of the full radar cloud and the camera image.
  // They keep the original simulator stamps and raw payload bytes (transport
  // adapters, not synthesized measurements) for setups where the Gazebo bridge
  // in sim_assets.launch.py does not carry these topics, e.g. secondary radar
  // instances. Enabling them alongside that bridge duplicates the topics.
  if (this->publish_ros_sensor_streams_)
  {
    this->full_cloud_ros_publisher_ = this->ros_node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        // Reliable, depth 100: a depth-1 best-effort publisher dropped clouds under load.
        this->full_topic_, rclcpp::QoS(100).reliable());
  }
  if (this->publish_camera_)
  {
    this->camera_mask_ros_publisher_ = this->ros_node_->create_publisher<sensor_msgs::msg::Image>(
        this->camera_mask_topic_, rclcpp::QoS(100).reliable());
    if (this->publish_ros_sensor_streams_)
    {
      this->camera_image_ros_publisher_ = this->ros_node_->create_publisher<sensor_msgs::msg::Image>(
          this->camera_image_topic_, rclcpp::QoS(100).reliable());
    }
    if (this->publish_camera_info_)
    {
      this->camera_info_publisher_ = this->ros_node_->create_publisher<sensor_msgs::msg::CameraInfo>(
          this->camera_info_topic_, rclcpp::QoS(100).reliable());
    }
    this->camera_truth_publisher_ = this->ros_node_->create_publisher<
        iii_drone_interfaces::msg::CameraFrameGroundTruth>(
        "/simulation/ground_truth/cable_camera/frame", rclcpp::QoS(100).reliable());
    if (!this->pylons_.empty())
    {
      this->pylon_mask_ros_publisher_ = this->ros_node_->create_publisher<sensor_msgs::msg::Image>(
          this->pylon_mask_topic_, rclcpp::QoS(100).reliable());
      this->pylon_camera_truth_publisher_ = this->ros_node_->create_publisher<
          iii_drone_interfaces::msg::PylonCameraFrameGroundTruth>(
          "/simulation/ground_truth/cable_camera/pylon_frame", rclcpp::QoS(100).reliable());
      this->pylon_exact_mask_truth_publisher_ = this->ros_node_->create_publisher<
          iii_drone_interfaces::msg::PylonExactMaskFrameGroundTruth>(
          "/simulation/ground_truth/cable_camera/pylon_exact_frame", rclcpp::QoS(100).reliable());
    }
  }
  if (this->publish_static_geometry_ || this->publish_camera_)
  {
    this->static_geometry_publisher_ = this->ros_node_->create_publisher<
        iii_drone_interfaces::msg::StaticConductorGeometry>(
        "/simulation/ground_truth/conductors/geometry",
        rclcpp::QoS(1).reliable().transient_local());
  }
  if (this->publish_static_geometry_)
  {
    this->PublishStaticGeometry(ToRosTime(std::chrono::steady_clock::duration::zero()));
  }
  if (this->aop_mode_)
  {
    this->truth_v2_publisher_ = this->ros_node_->create_publisher<
        iii_drone_interfaces::msg::RadarScanTruthV2>(this->truth_v2_topic_, rclcpp::QoS(1000).reliable());
    std::vector<aop_radar::Conductor> conductors;
    for (const auto & conductor : this->conductors_)
    {
      aop_radar::Conductor c;
      for (const auto & s : conductor.samples_world) {c.samples_world.emplace_back(s.X(), s.Y(), s.Z());}
      conductors.push_back(std::move(c));
    }
    std::vector<aop_radar::VoxelScatterer> voxels;
    std::vector<aop_radar::MemberSegment> members;   // r22: optional pylon lattice members
    try
    {
      voxels = aop_radar::load_scatterers_file(this->aop_config_.scatterers_path);
      members = aop_radar::load_pylon_members_file(this->aop_config_.scatterers_path);
    }
    catch (const std::exception & error)
    {
      gzerr << "MmwaveConductorSensorPlugin [" << this->radar_instance_ << "] failed to load scatterers: "
            << error.what() << "\n";
      return;
    }
    this->aop_model_.configure(
        this->aop_config_.profile, this->aop_config_.antenna, this->aop_config_.calibration,
        this->aop_config_.processing, std::move(conductors), std::move(voxels),
        aop_radar::mix(this->simulation_seed_, this->radar_seed_), this->aop_config_.scene_seed);
    this->aop_model_.set_pylon_members(std::move(members));
    // r22 Backlog 03: candidate-stage backend (config processing.backend, overridable by III_AOP_BACKEND)
    {
      std::string requested = this->aop_config_.backend;
      if (const char * env = std::getenv("III_AOP_BACKEND")) {if (*env) {requested = env;}}
      std::string note;
      try
      {
        this->aop_backend_ = aop_radar::attach_backend(this->aop_model_, requested, &note);
      }
      catch (const std::exception & error)
      {
        gzerr << "MmwaveConductorSensorPlugin [" << this->radar_instance_ << "] backend: " << error.what() << "\n";
        return;
      }
      gzmsg << "MmwaveConductorSensorPlugin [" << this->radar_instance_ << "] simulator-v2 backend requested=" << requested
            << " active=" << this->aop_backend_ << (note.empty() ? "" : " (" + note + ")") << "\n";
    }
    if (!this->aop_model_.configured())
    {
      gzerr << "MmwaveConductorSensorPlugin [" << this->radar_instance_ << "] AOP model not configured\n";
      return;
    }
  }

  this->link_.EnableVelocityChecks(ecm);

  this->publisher_ = this->transport_node_.Advertise<gz::msgs::PointCloudPacked>(
      this->topic_);
  this->full_publisher_ = this->transport_node_.Advertise<gz::msgs::PointCloudPacked>(
      this->full_topic_);
  this->label_publisher_ = this->transport_node_.Advertise<gz::msgs::PointCloudPacked>(
      this->label_topic_);
  if (this->publish_camera_)
  {
    this->conductor_id_map_publisher_ = this->transport_node_.Advertise<gz::msgs::StringMsg>(
        this->conductor_id_map_topic_);
  }
  if (this->publish_camera_ && !this->transport_node_.Subscribe(
      this->camera_image_topic_, &MmwaveConductorSensorPlugin::OnCameraImage, this))
  {
    gzerr << "MmwaveConductorSensorPlugin failed to subscribe to camera topic ["
          << this->camera_image_topic_ << "].\n";
    return;
  }
  if (this->publish_camera_ && !this->pylons_.empty() && !this->transport_node_.Subscribe(
      this->pylon_semantic_topic_,
      &MmwaveConductorSensorPlugin::OnPylonSemanticImage, this))
  {
    gzerr << "MmwaveConductorSensorPlugin failed to subscribe to evaluator pylon semantic topic ["
          << this->pylon_semantic_topic_ << "].\n";
    return;
  }
  gzmsg << "MmwaveConductorSensorPlugin configured for link [" << this->link_name_
        << "] with " << this->conductors_.size() << " conductors and "
        << this->pylons_.size() << " evaluator-only pylons on topics ["
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

  if (this->publish_drone_state_ &&
      (this->last_state_publish_time_ == std::chrono::steady_clock::duration::min() ||
      info.simTime - this->last_state_publish_time_ >= this->state_publish_period_))
  {
    this->PublishDroneState(info.simTime, ecm, link_world_pose.value());
    this->last_state_publish_time_ =
        this->last_state_publish_time_ == std::chrono::steady_clock::duration::min() ?
        info.simTime : this->last_state_publish_time_ + this->state_publish_period_;
  }

  if (this->aop_mode_)
  {
    this->AopUpdate(info, ecm, link_world_pose.value());
    return;
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
    gz::math::Vector3d nearest_physical_point_sensor;
    double nearest_physical_range = std::numeric_limits<double>::infinity();
    double conductor_length_prefix = 0.0;

    for (std::size_t i = 1; i < conductor.samples_world.size(); ++i)
    {
      const auto start_sensor = ToSensorFrame(sensor_world_pose, conductor.samples_world[i - 1]);
      const auto end_sensor = ToSensorFrame(sensor_world_pose, conductor.samples_world[i]);
      const auto segment_sensor = end_sensor - start_sensor;
      const double segment_length = segment_sensor.Length();
      if (segment_length < 1e-6)
      {
        continue;
      }
      const double nearest_parameter = std::clamp(
          -start_sensor.Dot(segment_sensor) / segment_sensor.SquaredLength(), 0.0, 1.0);
      const auto nearest_point = start_sensor + segment_sensor * nearest_parameter;
      if (nearest_point.Length() < nearest_physical_range)
      {
        nearest_physical_point_sensor = nearest_point;
        nearest_physical_range = nearest_point.Length();
      }

      auto candidate = this->ProjectClosestPointOnSegment(
          sensor_world_pose,
          conductor.samples_world[i - 1],
          conductor.samples_world[i]);

      if (candidate.has_value() && (!best_candidate.has_value() ||
          candidate->range < best_candidate->range))
      {
        candidate->conductor_parameter =
            conductor_length_prefix + candidate->segment_parameter * segment_length;
        best_candidate = std::move(candidate);
      }
      conductor_length_prefix += segment_length;
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

    if (!this->finite_fov_ && !this->IsInFov(noisy_point_sensor))
    {
      continue;
    }

    auto point = this->AddRadarSideInformation(
        noisy_point_sensor, sensor_velocity_sensor);
    point.ideal_generating_point_sensor = best_candidate->point_sensor;
    point.ideal_generating_point_world =
        sensor_world_pose.Pos() + sensor_world_pose.Rot().RotateVector(best_candidate->point_sensor);
    point.nearest_physical_point_sensor = nearest_physical_point_sensor;
    point.nearest_physical_point_world = sensor_world_pose.Pos() +
        sensor_world_pose.Rot().RotateVector(nearest_physical_point_sensor);
    point.physical_conductor_id = conductor.id;
    point.conductor_label = static_cast<std::uint16_t>(conductor_index + 1u);
    point.conductor_parameter = best_candidate->conductor_parameter;
    point.active_support_boundaries = best_candidate->active_support_boundaries;
    point.generating_point_equals_nearest_point =
        (best_candidate->point_sensor - nearest_physical_point_sensor).Length() <= 1e-7;
    point.generating_geometry_class = static_cast<std::uint8_t>(
        point.generating_point_equals_nearest_point ?
        mmwave_fov::GeneratingGeometryClass::ORTHOGONAL_NEAREST :
        mmwave_fov::GeneratingGeometryClass::FOV_LIMITED);
    point.unconstrained_nearest_distance_m = nearest_physical_range;
    point.line_of_sight_tangent_angle_rad =
        best_candidate->line_of_sight_tangent_angle_rad;
    points.push_back(std::move(point));
  }

  // Development-only pylon support is a bounded physical proxy: one vertical
  // support segment per source-mapped pylon.  It reuses the configured FOV,
  // range, Doppler, SNR, and noise model, while publishing a distinct
  // evaluator identity so no pylon return can be mistaken for a conductor.
  if (this->pylon_returns_enabled_)
  {
    for (const auto & pylon : this->pylons_)
    {
      const double center_x = 0.5 * (pylon.aabb_min_world.X() + pylon.aabb_max_world.X());
      const double center_y = 0.5 * (pylon.aabb_min_world.Y() + pylon.aabb_max_world.Y());
      const gz::math::Vector3d lower_world{center_x, center_y, pylon.aabb_min_world.Z()};
      const gz::math::Vector3d upper_world{center_x, center_y, pylon.aabb_max_world.Z()};
      auto candidate = this->ProjectClosestPointOnSegment(
          sensor_world_pose, lower_world, upper_world);
      if (!candidate.has_value() ||
          this->unit_uniform_(this->random_generator_) < this->dropout_probability_)
      {
        continue;
      }
      const auto noisy_point_sensor = this->ApplyMeasurementNoise(candidate.value());
      if (!this->finite_fov_ && !this->IsInFov(noisy_point_sensor))
      {
        continue;
      }
      auto point = this->AddRadarSideInformation(noisy_point_sensor, sensor_velocity_sensor);
      point.ideal_generating_point_sensor = candidate->point_sensor;
      point.ideal_generating_point_world = sensor_world_pose.Pos() +
          sensor_world_pose.Rot().RotateVector(candidate->point_sensor);
      point.nearest_physical_point_sensor = candidate->point_sensor;
      point.nearest_physical_point_world = point.ideal_generating_point_world;
      point.physical_pylon_id = pylon.id;
      point.conductor_label = 0u;
      point.source_class = iii_drone_interfaces::msg::RadarPointSource::VALID_PHYSICAL_PYLON;
      point.generating_point_equals_nearest_point = true;
      point.generating_geometry_class = static_cast<std::uint8_t>(
          mmwave_fov::GeneratingGeometryClass::ORTHOGONAL_NEAREST);
      point.active_support_boundaries = candidate->active_support_boundaries;
      point.unconstrained_nearest_distance_m = candidate->range;
      point.line_of_sight_tangent_angle_rad = candidate->line_of_sight_tangent_angle_rad;
      points.push_back(std::move(point));
    }
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

bool MmwaveConductorSensorPlugin::LoadPylons(const std::string & asset_uri)
{
  this->pylons_.clear();
  if (asset_uri.empty())
  {
    return true;
  }

  const std::string asset_path = gz::common::findFile(asset_uri);
  if (asset_path.empty())
  {
    gzerr << "Unable to resolve pylon evaluator asset [" << asset_uri << "].\n";
    return false;
  }

  const YAML::Node root = YAML::LoadFile(asset_path);
  const YAML::Node pylons_node = root["pylons"];
  const YAML::Node revision_node = root["geometry_revision"];
  if (!pylons_node || !pylons_node.IsSequence() || !revision_node || !revision_node.IsScalar())
  {
    gzerr << "Pylon evaluator asset [" << asset_path
          << "] does not contain a valid pylon map and geometry revision.\n";
    return false;
  }

  this->pylons_.reserve(pylons_node.size());
  for (const auto & pylon_node : pylons_node)
  {
    const YAML::Node id_node = pylon_node["id"];
    const YAML::Node min_node = pylon_node["aabb_min_world"];
    const YAML::Node max_node = pylon_node["aabb_max_world"];
    if (!id_node || !id_node.IsScalar() || !min_node || !max_node ||
        !min_node.IsSequence() || !max_node.IsSequence() ||
        min_node.size() != 3 || max_node.size() != 3)
    {
      return false;
    }
    Pylon pylon;
    pylon.id = id_node.as<std::string>();
    pylon.aabb_min_world = gz::math::Vector3d(
        min_node[0].as<double>(), min_node[1].as<double>(), min_node[2].as<double>());
    pylon.aabb_max_world = gz::math::Vector3d(
        max_node[0].as<double>(), max_node[1].as<double>(), max_node[2].as<double>());
    if (pylon.id.empty() || pylon.aabb_min_world.X() >= pylon.aabb_max_world.X() ||
        pylon.aabb_min_world.Y() >= pylon.aabb_max_world.Y() ||
        pylon.aabb_min_world.Z() >= pylon.aabb_max_world.Z())
    {
      return false;
    }
    this->pylons_.push_back(std::move(pylon));
  }
  this->pylon_geometry_revision_ = revision_node.as<std::string>();
  return this->pylons_.size() == 2u;
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

  if (!this->finite_fov_)
  {
    // VIEW_CONE: the unconstrained closest point on the segment, kept only if
    // it lies inside the cone.
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
    candidate.segment_parameter = interpolation;
    if (candidate.range > 1e-12)
    {
      candidate.line_of_sight_tangent_angle_rad = std::acos(std::clamp(
          std::abs(point_sensor.Dot(candidate.tangent_sensor)) / candidate.range, 0.0, 1.0));
    }
    return candidate;
  }

  const mmwave_fov::SensorFov fov{
      this->min_point_dist_, this->max_point_dist_, this->azimuth_half_angle_rad_,
      this->elevation_half_angle_rad_};
  const auto projection = mmwave_fov::ConstrainedClosestPointOnSegment(
      {start_sensor.X(), start_sensor.Y(), start_sensor.Z()},
      {end_sensor.X(), end_sensor.Y(), end_sensor.Z()}, fov);
  if (!projection.has_value())
  {
    return std::nullopt;
  }

  DetectionCandidate candidate;
  candidate.point_sensor = gz::math::Vector3d(
      projection->point_sensor.x, projection->point_sensor.y, projection->point_sensor.z);
  candidate.tangent_sensor = segment_sensor / segment_length;
  candidate.range = projection->range;
  candidate.segment_parameter = projection->parameter;
  candidate.active_support_boundaries = projection->active_boundaries;
  candidate.line_of_sight_tangent_angle_rad = projection->line_of_sight_tangent_angle_rad;
  return candidate;
}

bool MmwaveConductorSensorPlugin::IsInFov(
    const gz::math::Vector3d & point_sensor) const
{
  if (!this->finite_fov_)
  {
    const double range = point_sensor.Length();
    if (range > this->max_point_dist_)
    {
      return false;
    }

    const double yz_distance = std::hypot(point_sensor.Y(), point_sensor.Z());
    return point_sensor.X() > this->view_cone_slope_ * yz_distance;
  }

  return mmwave_fov::IsVisible(
      {point_sensor.X(), point_sensor.Y(), point_sensor.Z()},
      {this->min_point_dist_, this->max_point_dist_, this->azimuth_half_angle_rad_,
       this->elevation_half_angle_rad_});
}

gz::math::Vector3d MmwaveConductorSensorPlugin::ApplyMeasurementNoise(
    const DetectionCandidate & candidate)
{
  // FINITE_RECTANGULAR: visibility determines the physical source point before
  // this function is entered, so noise is not bounded by the segment. VIEW_CONE
  // keeps the along-conductor noise on the segment and re-checks visibility
  // after noise.
  double along_noise = this->unit_normal_(this->random_generator_) * this->sigma_along_m_;
  if (!this->finite_fov_)
  {
    along_noise = std::clamp(
        along_noise, -candidate.max_backward_span, candidate.max_forward_span);
  }

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

  if (this->full_cloud_ros_publisher_)
  {
    sensor_msgs::msg::PointCloud2 ros_full_message;
    ros_full_message.header.stamp = ToRosTime(gz::msgs::Convert(sim_time));
    ros_full_message.header.frame_id = this->frame_id_;
    ros_full_message.height = full_message.height();
    ros_full_message.width = full_message.width();
    ros_full_message.is_bigendian = false;
    ros_full_message.is_dense = full_message.is_dense();
    ros_full_message.point_step = full_message.point_step();
    ros_full_message.row_step = full_message.row_step();
    const auto add_field = [&ros_full_message](const char * name, std::uint32_t offset)
      {
        sensor_msgs::msg::PointField field;
        field.name = name;
        field.offset = offset;
        field.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field.count = 1;
        ros_full_message.fields.push_back(std::move(field));
      };
    add_field("x", 0);
    add_field("y", sizeof(float));
    add_field("z", 2 * sizeof(float));
    add_field("velocity", 3 * sizeof(float));
    add_field("snr", 4 * sizeof(float));
    add_field("noise", 5 * sizeof(float));
    ros_full_message.data.assign(full_message.data().begin(), full_message.data().end());
    this->full_cloud_ros_publisher_->publish(ros_full_message);
  }

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
    source.physical_pylon_id = point.physical_pylon_id;
    source.ideal_generating_point_world = ToRosPoint(point.ideal_generating_point_world);
    source.ideal_generating_point_sensor = ToRosPoint(point.ideal_generating_point_sensor);
    source.nearest_physical_point_world = ToRosPoint(point.nearest_physical_point_world);
    source.nearest_physical_point_sensor = ToRosPoint(point.nearest_physical_point_sensor);
    source.generating_point_equals_nearest_point = point.generating_point_equals_nearest_point;
    source.conductor_parameter_m = point.conductor_parameter;
    source.generating_geometry_class = point.generating_geometry_class;
    source.active_support_boundaries = point.active_support_boundaries;
    source.unconstrained_nearest_distance_m = point.unconstrained_nearest_distance_m;
    source.line_of_sight_tangent_angle_rad = point.line_of_sight_tangent_angle_rad;
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
    const gz::math::Pose3d & link_world_pose,
    const std::uint64_t frame_sequence)
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

  if (this->camera_info_publisher_)
  {
    // Calibration is emitted on the same simulator source timestamp as the
    // rendered image/mask.  It is an explicit bag topic rather than an
    // out-of-band runtime default, so an offline replay can bind each frame to
    // the calibrated project-owned camera model.
    sensor_msgs::msg::CameraInfo camera_info;
    camera_info.header = ros_mask.header;
    camera_info.width = this->camera_width_;
    camera_info.height = this->camera_height_;
    camera_info.distortion_model = "plumb_bob";
    camera_info.d = {0.0, 0.0, 0.0, 0.0, 0.0};
    camera_info.k = {focal_length, 0.0, center_x, 0.0, focal_length, center_y, 0.0, 0.0, 1.0};
    camera_info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    camera_info.p = {focal_length, 0.0, center_x, 0.0,
                     0.0, focal_length, center_y, 0.0,
                     0.0, 0.0, 1.0, 0.0};
    this->camera_info_publisher_->publish(camera_info);
  }

  iii_drone_interfaces::msg::CameraFrameGroundTruth frame_truth;
  frame_truth.header = ros_mask.header;
  frame_truth.source_image_topic = this->camera_image_topic_;
  frame_truth.instance_mask_topic = this->camera_mask_topic_;
  frame_truth.frame_sequence = frame_sequence;
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

  const auto stamp_ns = StampNanoseconds(image.header().stamp());
  std::uint64_t frame_sequence{};
  std::optional<PendingPylonFrame> pending_pylon;
  {
    std::lock_guard<std::mutex> lock(this->camera_registration_mutex_);
    frame_sequence = this->camera_frame_sequence_++;
    // Evaluator pylon frames are registered to RGB frames by source stamp,
    // only when a pylon map is configured.
    if (!this->pylons_.empty())
    {
      const auto pending = this->pending_pylon_frames_.find(stamp_ns);
      if (pending != this->pending_pylon_frames_.end())
      {
        pending_pylon = std::move(pending->second);
        this->pending_pylon_frames_.erase(pending);
      }
      else
      {
        this->camera_frame_identities_[stamp_ns] = frame_sequence;
      }
      while (this->camera_frame_identities_.size() > kCameraRegistrationWindowFrames)
      {
        RCLCPP_ERROR(
          this->ros_node_->get_logger(),
          "Dropping unmatched RGB frame identity at source stamp %lld; registration fails closed.",
          static_cast<long long>(this->camera_frame_identities_.begin()->first));
        this->camera_frame_identities_.erase(this->camera_frame_identities_.begin());
      }
    }
  }
  if (this->camera_image_ros_publisher_)
  {
    this->PublishRosCameraImage(image);
  }
  this->PublishCameraGroundTruth(image.header().stamp(), link_world_pose, frame_sequence);
  if (pending_pylon)
  {
    this->PublishPylonSemanticImage(
      pending_pylon->image, pending_pylon->link_world_pose, frame_sequence);
  }
}

void MmwaveConductorSensorPlugin::PublishRosCameraImage(const gz::msgs::Image & image)
{
  sensor_msgs::msg::Image ros_image;
  ros_image.header.stamp = ToRosTime(image.header().stamp());
  ros_image.header.frame_id = "cable_camera";
  ros_image.width = image.width();
  ros_image.height = image.height();
  ros_image.is_bigendian = false;
  ros_image.step = image.step();
  switch (image.pixel_format_type())
  {
    case gz::msgs::PixelFormatType::L_INT8:
      ros_image.encoding = "mono8";
      break;
    case gz::msgs::PixelFormatType::RGB_INT8:
      ros_image.encoding = "rgb8";
      break;
    case gz::msgs::PixelFormatType::BGR_INT8:
      ros_image.encoding = "bgr8";
      break;
    case gz::msgs::PixelFormatType::RGBA_INT8:
      ros_image.encoding = "rgba8";
      break;
    case gz::msgs::PixelFormatType::BGRA_INT8:
      ros_image.encoding = "bgra8";
      break;
    default:
      gzerr << "MmwaveConductorSensorPlugin cannot relay camera pixel format ["
            << image.pixel_format_type() << "] to ROS Image.\n";
      return;
  }
  ros_image.data.assign(image.data().begin(), image.data().end());
  this->camera_image_ros_publisher_->publish(ros_image);
}

void MmwaveConductorSensorPlugin::OnPylonSemanticImage(const gz::msgs::Image & image)
{
  if (image.width() != this->camera_width_ || image.height() != this->camera_height_ ||
      image.step() < image.width() * 3u ||
      (image.pixel_format_type() != gz::msgs::PixelFormatType::RGB_INT8 &&
       image.pixel_format_type() != gz::msgs::PixelFormatType::BGR_INT8))
  {
    gzerr << "Evaluator pylon semantic image violates the registered camera contract: "
          << image.width() << "x" << image.height() << ", step=" << image.step()
          << ", format=" << image.pixel_format_type() << ".\n";
    return;
  }

  if (image.data().size() < static_cast<std::size_t>(image.step()) * image.height())
  {
    gzerr << "Evaluator pylon semantic image payload is truncated.\n";
    return;
  }

  gz::math::Pose3d link_world_pose;
  {
    std::lock_guard<std::mutex> lock(this->camera_pose_mutex_);
    if (!this->has_latest_link_world_pose_)
    {
      return;
    }
    link_world_pose = this->latest_link_world_pose_;
  }

  const auto stamp_ns = StampNanoseconds(image.header().stamp());
  std::optional<std::uint64_t> frame_sequence;
  {
    std::lock_guard<std::mutex> lock(this->camera_registration_mutex_);
    const auto identity = this->camera_frame_identities_.find(stamp_ns);
    if (identity != this->camera_frame_identities_.end())
    {
      frame_sequence = identity->second;
      this->camera_frame_identities_.erase(identity);
    }
    else
    {
      this->pending_pylon_frames_[stamp_ns] = PendingPylonFrame{image, link_world_pose};
      while (this->pending_pylon_frames_.size() > kCameraRegistrationWindowFrames)
      {
        RCLCPP_ERROR(
          this->ros_node_->get_logger(),
          "Dropping unmatched evaluator frame at source stamp %lld; registration fails closed.",
          static_cast<long long>(this->pending_pylon_frames_.begin()->first));
        this->pending_pylon_frames_.erase(this->pending_pylon_frames_.begin());
      }
      return;
    }
  }
  this->PublishPylonSemanticImage(image, link_world_pose, *frame_sequence);
}

void MmwaveConductorSensorPlugin::PublishPylonSemanticImage(
    const gz::msgs::Image & image,
    const gz::math::Pose3d & link_world_pose,
    const std::uint64_t frame_sequence)
{
  const std::size_t pixel_count =
      static_cast<std::size_t>(this->camera_width_) * this->camera_height_;
  const gz::math::Pose3d camera_world_pose = link_world_pose * this->camera_pose_;
  const double focal_length =
      0.5 * static_cast<double>(this->camera_width_) /
      std::tan(0.5 * this->camera_horizontal_fov_rad_);
  const double center_x = 0.5 * (static_cast<double>(this->camera_width_) - 1.0);
  const double center_y = 0.5 * (static_cast<double>(this->camera_height_) - 1.0);

  std::vector<std::uint16_t> labels(pixel_count, 0u);
  for (std::uint32_t v = 0; v < this->camera_height_; ++v)
  {
    for (std::uint32_t u = 0; u < this->camera_width_; ++u)
    {
      const std::size_t source_offset =
          static_cast<std::size_t>(v) * image.step() + static_cast<std::size_t>(u) * 3u;
      const auto label = static_cast<std::uint8_t>(image.data()[source_offset]);
      // Labels 1 and 2 are reserved for the two physical pylons.  Every
      // unlabelled renderer entity remains evaluator background.
      if (label <= this->pylons_.size())
      {
        labels[static_cast<std::size_t>(v) * this->camera_width_ + u] = label;
      }
    }
  }

  sensor_msgs::msg::Image ros_mask;
  ros_mask.header.stamp = ToRosTime(image.header().stamp());
  ros_mask.header.frame_id = "cable_camera";
  ros_mask.width = this->camera_width_;
  ros_mask.height = this->camera_height_;
  ros_mask.encoding = "mono16";
  ros_mask.is_bigendian = false;
  ros_mask.step = this->camera_width_ * sizeof(std::uint16_t);
  ros_mask.data.resize(labels.size() * sizeof(std::uint16_t));
  std::memcpy(ros_mask.data.data(), labels.data(), ros_mask.data.size());
  this->pylon_mask_ros_publisher_->publish(ros_mask);

  iii_drone_interfaces::msg::PylonExactMaskFrameGroundTruth truth;
  truth.header = ros_mask.header;
  truth.source_image_topic = this->camera_image_topic_;
  truth.exact_instance_mask_topic = this->pylon_mask_topic_;
  truth.frame_sequence = frame_sequence;
  truth.image_width = this->camera_width_;
  truth.image_height = this->camera_height_;
  truth.pylon_geometry_revision = this->pylon_geometry_revision_;
  truth.renderer_contract =
      "gazebo_ogre2_semantic_labels_same_pose_projection_rate_clip_and_source_stamp";
  truth.pylons.reserve(this->pylons_.size());
  iii_drone_interfaces::msg::PylonCameraFrameGroundTruth legacy_truth;
  legacy_truth.header = ros_mask.header;
  legacy_truth.source_image_topic = this->camera_image_topic_;
  legacy_truth.frame_sequence = truth.frame_sequence;
  legacy_truth.image_width = this->camera_width_;
  legacy_truth.image_height = this->camera_height_;
  legacy_truth.pylon_geometry_revision = this->pylon_geometry_revision_;
  legacy_truth.pylons.reserve(this->pylons_.size());

  for (std::size_t pylon_index = 0; pylon_index < this->pylons_.size(); ++pylon_index)
  {
    const auto & pylon = this->pylons_[pylon_index];
    const auto label = static_cast<std::uint16_t>(pylon_index + 1u);
    iii_drone_interfaces::msg::PylonExactMaskVisibility visibility;
    visibility.physical_pylon_id = pylon.id;
    visibility.exact_instance_mask_available = true;
    visibility.visibility_method = "renderer_exact_visible_mesh_semantic_mask";
    visibility.mask_value = label;

    const auto center_world = 0.5 * (pylon.aabb_min_world + pylon.aabb_max_world);
    const auto center_camera = ToSensorFrame(camera_world_pose, center_world);
    visibility.range_m = center_camera.Length();
    visibility.bearing_rad = std::atan2(center_camera.Y(), center_camera.X());

    bool has_front_corner = false;
    bool has_behind_corner = false;
    double projected_min_u = std::numeric_limits<double>::infinity();
    double projected_min_v = std::numeric_limits<double>::infinity();
    double projected_max_u = -std::numeric_limits<double>::infinity();
    double projected_max_v = -std::numeric_limits<double>::infinity();
    for (const double x : {pylon.aabb_min_world.X(), pylon.aabb_max_world.X()})
    {
      for (const double y : {pylon.aabb_min_world.Y(), pylon.aabb_max_world.Y()})
      {
        for (const double z : {pylon.aabb_min_world.Z(), pylon.aabb_max_world.Z()})
        {
          const auto point_camera = ToSensorFrame(camera_world_pose, {x, y, z});
          if (point_camera.X() <= 0.02)
          {
            has_behind_corner = true;
            continue;
          }
          has_front_corner = true;
          const double u = center_x - focal_length * point_camera.Y() / point_camera.X();
          const double v = center_y - focal_length * point_camera.Z() / point_camera.X();
          projected_min_u = std::min(projected_min_u, u);
          projected_min_v = std::min(projected_min_v, v);
          projected_max_u = std::max(projected_max_u, u);
          projected_max_v = std::max(projected_max_v, v);
        }
      }
    }

    double projected_area = 0.0;
    bool projected_intersects_image = false;
    if (has_front_corner)
    {
      const double projected_width =
          std::max(0.0, std::ceil(projected_max_u) - std::floor(projected_min_u) + 1.0);
      const double projected_height =
          std::max(0.0, std::ceil(projected_max_v) - std::floor(projected_min_v) + 1.0);
      projected_area = projected_width * projected_height;
      visibility.projected_bbox_pixel_count = static_cast<std::uint32_t>(std::min(
          projected_area, static_cast<double>(std::numeric_limits<std::uint32_t>::max())));
      const double clipped_min_u = std::max(0.0, std::ceil(projected_min_u));
      const double clipped_min_v = std::max(0.0, std::ceil(projected_min_v));
      const double clipped_max_u = std::min(
          static_cast<double>(this->camera_width_ - 1u), std::floor(projected_max_u));
      const double clipped_max_v = std::min(
          static_cast<double>(this->camera_height_ - 1u), std::floor(projected_max_v));
      projected_intersects_image =
          clipped_min_u <= clipped_max_u && clipped_min_v <= clipped_max_v;
      if (projected_intersects_image && projected_area > 0.0)
      {
        const double clipped_area =
            (clipped_max_u - clipped_min_u + 1.0) * (clipped_max_v - clipped_min_v + 1.0);
        visibility.clipping_fraction = 1.0 - clipped_area / projected_area;
      }
      else
      {
        visibility.clipping_fraction = 1.0;
      }
    }
    else
    {
      visibility.clipping_fraction = 1.0;
    }

    std::uint32_t min_u = this->camera_width_;
    std::uint32_t min_v = this->camera_height_;
    std::uint32_t max_u = 0u;
    std::uint32_t max_v = 0u;
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

    visibility.has_visible_bounding_box = visibility.visible_pixel_count > 0u;
    visibility.image_area_fraction =
        static_cast<double>(visibility.visible_pixel_count) / static_cast<double>(pixel_count);
    if (visibility.has_visible_bounding_box)
    {
      visibility.min_u = min_u;
      visibility.min_v = min_v;
      visibility.max_u = max_u;
      visibility.max_v = max_v;
      visibility.visible_bbox_pixel_count = (max_u - min_u + 1u) * (max_v - min_v + 1u);
      visibility.visible_bbox_fraction = projected_area > 0.0 ?
          static_cast<double>(visibility.visible_bbox_pixel_count) / projected_area : 0.0;
      const bool mask_touches_frame = min_u == 0u || min_v == 0u ||
          max_u + 1u == this->camera_width_ || max_v + 1u == this->camera_height_;
      visibility.visibility_state =
          (has_behind_corner || visibility.clipping_fraction > 0.0 || mask_touches_frame) ?
          iii_drone_interfaces::msg::PylonExactMaskVisibility::PARTIALLY_CLIPPED :
          iii_drone_interfaces::msg::PylonExactMaskVisibility::PROJECTED_VISIBLE;
      visibility.occlusion_status = "renderer_resolved_visible_surface";
    }
    else if (!has_front_corner)
    {
      visibility.visibility_state =
          iii_drone_interfaces::msg::PylonExactMaskVisibility::BEHIND_CAMERA;
      visibility.occlusion_status = "renderer_exact_no_visible_pixels_behind_camera";
    }
    else if (!projected_intersects_image)
    {
      visibility.visibility_state =
          iii_drone_interfaces::msg::PylonExactMaskVisibility::OUTSIDE_FOV;
      visibility.occlusion_status = "renderer_exact_no_visible_pixels_outside_fov";
    }
    else
    {
      visibility.visibility_state =
          iii_drone_interfaces::msg::PylonExactMaskVisibility::OCCLUDED_OR_NO_VISIBLE_PIXELS;
      visibility.occlusion_status = "renderer_exact_no_visible_surface_pixels";
    }
    iii_drone_interfaces::msg::CameraPylonVisibility legacy;
    legacy.physical_pylon_id = visibility.physical_pylon_id;
    legacy.visibility_state = visibility.visibility_state ==
        iii_drone_interfaces::msg::PylonExactMaskVisibility::OCCLUDED_OR_NO_VISIBLE_PIXELS ?
        iii_drone_interfaces::msg::CameraPylonVisibility::PROJECTED_VISIBLE :
        visibility.visibility_state;
    legacy.exact_instance_mask_available = true;
    legacy.visibility_method = visibility.visibility_method;
    legacy.projected_bbox_pixel_count = visibility.projected_bbox_pixel_count;
    legacy.visible_bbox_pixel_count = visibility.visible_bbox_pixel_count;
    legacy.visible_bbox_fraction = visibility.visible_bbox_fraction;
    legacy.image_area_fraction = visibility.image_area_fraction;
    legacy.has_visible_bounding_box = visibility.has_visible_bounding_box;
    legacy.min_u = visibility.min_u;
    legacy.min_v = visibility.min_v;
    legacy.max_u = visibility.max_u;
    legacy.max_v = visibility.max_v;
    legacy.range_m = visibility.range_m;
    legacy.bearing_rad = visibility.bearing_rad;
    legacy.clipping_fraction = visibility.clipping_fraction;
    legacy.occlusion_status = visibility.occlusion_status;
    legacy_truth.pylons.push_back(std::move(legacy));
    truth.pylons.push_back(std::move(visibility));
  }
  this->pylon_exact_mask_truth_publisher_->publish(truth);
  this->pylon_camera_truth_publisher_->publish(legacy_truth);
}

// ---------------------------------------------------------------------------
// r21 / WO-2026-09-29-001 simulator-v2 (AOP_FAST_POINT) frame loop.
// Frame k of this radar starts at k * T + offset + jitter_k on the simulation clock (shared
// frame period T, deterministic per-radar trigger jitter).  Geometry is sampled at the first
// simulation step at or after the frame start; the cloud is stamped with that sampling time.
// The RF-schedule overlap with the configured peer window drives the interference model.
void MmwaveConductorSensorPlugin::AopUpdate(
    const gz::sim::UpdateInfo & info,
    const gz::sim::EntityComponentManager & ecm,
    const gz::math::Pose3d & link_world_pose)
{
  const auto & prof = this->aop_model_.profile();
  const double period_s = prof.frame_period_ms * 1e-3;
  const double sim_s = std::chrono::duration<double>(info.simTime).count();
  const double offset_s = this->schedule_offset_ms_ * 1e-3;
  const std::uint64_t stream = aop_radar::mix(this->simulation_seed_, this->radar_seed_);
  auto jitter_of = [&](std::uint64_t k) {
      if (this->schedule_jitter_sigma_us_ <= 0.0) {return 0.0;}
      aop_radar::Rng rng(aop_radar::mix(stream ^ 0x7A1D5EEDULL, k));
      return std::clamp(rng.normal(), -4.0, 4.0) * this->schedule_jitter_sigma_us_ * 1e-6;
    };
  const std::uint64_t k = this->aop_next_frame_;
  const double jitter_s = jitter_of(k);
  const double start_s = static_cast<double>(k) * period_s + offset_s + jitter_s;
  if (sim_s + 1e-9 < start_s)
  {
    return;
  }
  double overlap = 0.0;
  if (this->peer_offset_ms_ >= 0.0 && this->peer_active_ms_ > 0.0)
  {
    const double own0 = start_s, own1 = start_s + prof.frame_active_s;
    for (int shift = -1; shift <= 1; ++shift)
    {
      const double peer0 = (static_cast<double>(k) + shift) * period_s + this->peer_offset_ms_ * 1e-3;
      const double peer1 = peer0 + this->peer_active_ms_ * 1e-3;
      overlap += std::max(0.0, std::min(own1, peer1) - std::max(own0, peer0));
    }
    overlap = std::clamp(overlap / std::max(1e-12, prof.frame_active_s), 0.0, 1.0);
  }
  const gz::math::Pose3d sensor_world_pose = link_world_pose * this->sensor_pose_;
  const gz::math::Vector3d velocity_world =
      this->link_.WorldLinearVelocity(ecm, this->sensor_pose_.Pos()).value_or(gz::math::Vector3d::Zero);
  aop_radar::SensorState state;
  state.position_world = {sensor_world_pose.Pos().X(), sensor_world_pose.Pos().Y(), sensor_world_pose.Pos().Z()};
  const auto ex = sensor_world_pose.Rot().RotateVector(gz::math::Vector3d::UnitX);
  const auto ey = sensor_world_pose.Rot().RotateVector(gz::math::Vector3d::UnitY);
  const auto ez = sensor_world_pose.Rot().RotateVector(gz::math::Vector3d::UnitZ);
  state.rotation_world_from_sensor.m = {ex.X(), ey.X(), ez.X(), ex.Y(), ey.Y(), ez.Y(), ex.Z(), ey.Z(), ez.Z()};
  state.velocity_world = {velocity_world.X(), velocity_world.Y(), velocity_world.Z()};
  aop_radar::ScanDiagnostics diag;
  const auto detections = this->aop_model_.scan(state, k, overlap, &diag);
  this->PublishAopScan(info.simTime, detections, diag, sensor_world_pose, velocity_world, k, jitter_s, overlap);
  // advance to the next frame whose start lies after this step (no frame is emitted twice)
  std::uint64_t next = k + 1;
  while (static_cast<double>(next) * period_s + offset_s <= sim_s - period_s) {++next;}
  this->aop_next_frame_ = next;
}

void MmwaveConductorSensorPlugin::PublishAopScan(
    const std::chrono::steady_clock::duration & sim_time,
    const std::vector<aop_radar::Detection> & detections,
    const aop_radar::ScanDiagnostics & diag,
    const gz::math::Pose3d & sensor_world_pose,
    const gz::math::Vector3d & sensor_velocity_world,
    std::uint64_t frame_index, double jitter_s, double overlap)
{
  gz::msgs::PointCloudPacked message;
  gz::msgs::InitPointCloudPacked(message, this->frame_id_, false,
      {{"xyz", gz::msgs::PointCloudPacked::Field::FLOAT32}});
  gz::msgs::PointCloudPacked full_message;
  gz::msgs::InitPointCloudPacked(full_message, this->frame_id_, false,
      {{"xyz", gz::msgs::PointCloudPacked::Field::FLOAT32},
       {"velocity", gz::msgs::PointCloudPacked::Field::FLOAT32},
       {"snr", gz::msgs::PointCloudPacked::Field::FLOAT32},
       {"noise", gz::msgs::PointCloudPacked::Field::FLOAT32}});
  gz::msgs::PointCloudPacked label_message;
  gz::msgs::InitPointCloudPacked(label_message, this->frame_id_, false,
      {{"conductor_id", gz::msgs::PointCloudPacked::Field::UINT16}});
  *message.mutable_header()->mutable_stamp() = gz::msgs::Convert(sim_time);
  *full_message.mutable_header()->mutable_stamp() = gz::msgs::Convert(sim_time);
  *label_message.mutable_header()->mutable_stamp() = gz::msgs::Convert(sim_time);
  const auto n = static_cast<std::uint32_t>(detections.size());
  for (auto * m : {&message, &full_message, &label_message})
  {
    m->set_height(1);
    m->set_width(n);
    m->set_is_dense(true);
    m->set_row_step(m->point_step() * n);
    m->mutable_data()->resize(static_cast<std::size_t>(n) * m->point_step());
  }
  if (n > 0)
  {
    gz::msgs::PointCloudPackedIterator<float> x_it(message, "x"), y_it(message, "y"), z_it(message, "z");
    gz::msgs::PointCloudPackedIterator<float> fx(full_message, "x"), fy(full_message, "y"), fz(full_message, "z");
    gz::msgs::PointCloudPackedIterator<float> fv(full_message, "velocity"), fs(full_message, "snr"), fn(full_message, "noise");
    gz::msgs::PointCloudPackedIterator<std::uint16_t> lab(label_message, "conductor_id");
    for (const auto & d : detections)
    {
      // runtime-observable fields only
      *x_it = d.x; *y_it = d.y; *z_it = d.z;
      *fx = d.x; *fy = d.y; *fz = d.z; *fv = d.velocity; *fs = d.snr_db; *fn = d.noise_db;
      const bool conductor = d.mechanism == aop_radar::Mechanism::CONDUCTOR_SPECULAR ||
        d.mechanism == aop_radar::Mechanism::CONDUCTOR_DIFFUSE;
      *lab = conductor && d.source_index >= 0 ? static_cast<std::uint16_t>(d.source_index + 1) : 0u;
      ++x_it; ++y_it; ++z_it; ++fx; ++fy; ++fz; ++fv; ++fs; ++fn; ++lab;
    }
  }
  this->publisher_.Publish(message);
  this->full_publisher_.Publish(full_message);
  this->label_publisher_.Publish(label_message);

  std_msgs::msg::Header scan_header;
  scan_header.stamp = ToRosTime(gz::msgs::Convert(sim_time));
  scan_header.frame_id = this->frame_id_;
  if (this->full_cloud_ros_publisher_)
  {
    sensor_msgs::msg::PointCloud2 ros_full;
    ros_full.header = scan_header;
    ros_full.height = 1;
    ros_full.width = n;
    ros_full.is_bigendian = false;
    ros_full.is_dense = true;
    ros_full.point_step = full_message.point_step();
    ros_full.row_step = full_message.row_step();
    const char * names[] = {"x", "y", "z", "velocity", "snr", "noise"};
    for (std::uint32_t i = 0; i < 6; ++i)
    {
      sensor_msgs::msg::PointField field;
      field.name = names[i];
      field.offset = i * sizeof(float);
      field.datatype = sensor_msgs::msg::PointField::FLOAT32;
      field.count = 1;
      ros_full.fields.push_back(std::move(field));
    }
    ros_full.data.assign(full_message.data().begin(), full_message.data().end());
    this->full_cloud_ros_publisher_->publish(ros_full);
  }

  iii_drone_interfaces::msg::RadarScanTruthV2 truth;
  truth.header = scan_header;
  truth.radar_instance = this->radar_instance_;
  truth.radar_model = this->radar_model_;
  truth.profile_id = this->aop_config_.profile_id;
  truth.source_topic = this->full_topic_;
  truth.scan_sequence = this->aop_scan_sequence_++;
  truth.frame_index = frame_index;
  truth.frame_period_s = this->aop_model_.profile().frame_period_ms * 1e-3;
  truth.schedule_offset_s = this->schedule_offset_ms_ * 1e-3;
  truth.schedule_jitter_s = jitter_s;
  truth.schedule_overlap_fraction = overlap;
  truth.noise_rise_db = static_cast<float>(diag.noise_rise_db);
  truth.candidates = static_cast<std::uint32_t>(diag.candidates);
  truth.cells = static_cast<std::uint32_t>(diag.cells);
  truth.false_alarms = static_cast<std::uint32_t>(diag.false_alarms);
  truth.sensor_pose_world.position = ToRosPoint(sensor_world_pose.Pos());
  truth.sensor_pose_world.orientation.x = sensor_world_pose.Rot().X();
  truth.sensor_pose_world.orientation.y = sensor_world_pose.Rot().Y();
  truth.sensor_pose_world.orientation.z = sensor_world_pose.Rot().Z();
  truth.sensor_pose_world.orientation.w = sensor_world_pose.Rot().W();
  truth.sensor_velocity_world.x = sensor_velocity_world.X();
  truth.sensor_velocity_world.y = sensor_velocity_world.Y();
  truth.sensor_velocity_world.z = sensor_velocity_world.Z();
  truth.points.reserve(detections.size());
  for (std::size_t i = 0; i < detections.size(); ++i)
  {
    const auto & d = detections[i];
    iii_drone_interfaces::msg::RadarDetectionTruthV2 src;
    src.source_point_index = static_cast<std::uint32_t>(i);
    src.mechanism = static_cast<std::uint8_t>(d.mechanism);
    switch (d.mechanism)
    {
      case aop_radar::Mechanism::CONDUCTOR_SPECULAR:
      case aop_radar::Mechanism::CONDUCTOR_DIFFUSE:
        src.source_class = iii_drone_interfaces::msg::RadarPointSource::VALID_PHYSICAL_CONDUCTOR;
        if (d.source_index >= 0 && static_cast<std::size_t>(d.source_index) < this->conductors_.size())
        {
          src.physical_conductor_id = this->conductors_[static_cast<std::size_t>(d.source_index)].id;
        }
        break;
      case aop_radar::Mechanism::PYLON:
        src.source_class = iii_drone_interfaces::msg::RadarPointSource::VALID_PHYSICAL_PYLON;
        if (d.source_index >= 0 && static_cast<std::size_t>(d.source_index) < this->pylons_.size())
        {
          src.physical_pylon_id = this->pylons_[static_cast<std::size_t>(d.source_index)].id;
        }
        break;
      case aop_radar::Mechanism::MULTIPATH_GHOST:
        src.source_class = iii_drone_interfaces::msg::RadarPointSource::PHANTOM;
        break;
      default:
        src.source_class = iii_drone_interfaces::msg::RadarPointSource::CLUTTER_NO_PHYSICAL_SOURCE;
    }
    src.source_index = d.source_index;
    src.dominant_power_fraction = static_cast<float>(d.dominant_power_fraction);
    src.scatterers_in_cell = d.scatterers_in_cell;
    src.ideal_point_world.x = d.ideal_point_world.x;
    src.ideal_point_world.y = d.ideal_point_world.y;
    src.ideal_point_world.z = d.ideal_point_world.z;
    src.ideal_point_sensor.x = d.ideal_point_sensor.x;
    src.ideal_point_sensor.y = d.ideal_point_sensor.y;
    src.ideal_point_sensor.z = d.ideal_point_sensor.z;
    src.conductor_parameter_m = d.conductor_parameter_m;
    src.two_way_gain_db = static_cast<float>(d.two_way_gain_db);
    src.mean_snr_db = static_cast<float>(d.mean_snr_db);
    src.range_bin = d.range_bin;
    src.doppler_bin = d.doppler_bin;
    truth.points.push_back(std::move(src));
  }
  this->truth_v2_publisher_->publish(truth);
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
