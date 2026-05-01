#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
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
#include <gz/msgs/Utility.hh>
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

constexpr double kDefaultMinPointDist = 0.99;
constexpr double kDefaultMaxPointDist = 18.0;
constexpr double kDefaultViewConeSlope = 0.55;
constexpr double kDefaultPlaneHalfThickness = 0.08;
constexpr double kDefaultSigmaAlong = 0.12;
constexpr double kDefaultSigmaCross = 0.03;
constexpr double kDefaultSigmaNormal = 0.02;
constexpr double kDefaultDropoutProbability = 0.02;
constexpr double kDefaultCorridorMargin = 0.0;

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

  private: bool LoadConductors(const std::string & asset_uri);
  private: bool ResolveLink(const gz::sim::EntityComponentManager & ecm);
  private: void UpdateCorridorBounds();
  private: bool IsWithinCorridorFootprint(
      const gz::math::Vector3d & world_point) const;
  private: std::optional<DetectionCandidate> ProjectClosestPointOnSegment(
      const gz::math::Pose3d & sensor_world_pose,
      const gz::math::Vector3d & start_world,
      const gz::math::Vector3d & end_world) const;
  private: bool IsInFov(const gz::math::Vector3d & point_sensor) const;
  private: gz::math::Vector3d ApplyMeasurementNoise(
      const DetectionCandidate & candidate);
  private: void PublishPointCloud(
      const std::chrono::steady_clock::duration & sim_time,
      const std::vector<gz::math::Vector3d> & points);

  private: gz::sim::Entity model_entity_{gz::sim::kNullEntity};
  private: gz::sim::Entity link_entity_{gz::sim::kNullEntity};
  private: gz::sim::Link link_;
  private: std::string link_name_{"base_link"};
  private: std::string topic_{"/sensor/mmwave/points"};
  private: std::string frame_id_{"mmwave"};
  private: std::string conductor_asset_uri_{
      "model://hcaa_pylon_setup/conductors.yaml"};
  private: gz::math::Pose3d sensor_pose_{
      0.0, 0.0, 0.1, 3.1415, -1.57079632679, 0.0};
  private: double update_rate_hz_{30.0};
  private: double min_point_dist_{kDefaultMinPointDist};
  private: double max_point_dist_{kDefaultMaxPointDist};
  private: double view_cone_slope_{kDefaultViewConeSlope};
  private: double plane_half_thickness_m_{kDefaultPlaneHalfThickness};
  private: double sigma_along_m_{kDefaultSigmaAlong};
  private: double sigma_cross_m_{kDefaultSigmaCross};
  private: double sigma_normal_m_{kDefaultSigmaNormal};
  private: double dropout_probability_{kDefaultDropoutProbability};
  private: double corridor_margin_m_{kDefaultCorridorMargin};
  private: bool stamp_at_latest_tf_{true};
  private: std::chrono::steady_clock::duration publish_period_{std::chrono::milliseconds(33)};
  private: std::chrono::steady_clock::duration last_publish_time_{
      std::chrono::steady_clock::duration::min()};
  private: gz::transport::Node transport_node_;
  private: gz::transport::Node::Publisher publisher_;
  private: std::vector<Conductor> conductors_;
  private: gz::math::Vector3d corridor_origin_world_{gz::math::Vector3d::Zero};
  private: gz::math::Vector3d corridor_span_axis_world_{gz::math::Vector3d::UnitX};
  private: gz::math::Vector3d corridor_lateral_axis_world_{gz::math::Vector3d::UnitY};
  private: double corridor_min_span_{0.0};
  private: double corridor_max_span_{0.0};
  private: double corridor_min_lateral_{0.0};
  private: double corridor_max_lateral_{0.0};
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
  this->model_entity_ = entity;

  if (sdf->HasElement("link_name"))
  {
    this->link_name_ = sdf->Get<std::string>("link_name");
  }
  if (sdf->HasElement("topic"))
  {
    this->topic_ = sdf->Get<std::string>("topic");
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
  if (sdf->HasElement("update_rate_hz"))
  {
    this->update_rate_hz_ = sdf->Get<double>("update_rate_hz");
  }
  if (sdf->HasElement("min_point_dist"))
  {
    this->min_point_dist_ = sdf->Get<double>("min_point_dist");
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

  this->publisher_ = this->transport_node_.Advertise<gz::msgs::PointCloudPacked>(
      this->topic_);
  gzmsg << "MmwaveConductorSensorPlugin configured for link [" << this->link_name_
        << "] with " << this->conductors_.size() << " conductors on topic ["
        << this->topic_ << "].\n";
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

  if (this->publish_period_ > std::chrono::steady_clock::duration::zero() &&
      this->last_publish_time_ != std::chrono::steady_clock::duration::min() &&
      info.simTime - this->last_publish_time_ < this->publish_period_)
  {
    return;
  }

  const auto link_world_pose = this->link_.WorldPose(ecm);
  if (!link_world_pose.has_value())
  {
    return;
  }

  const gz::math::Pose3d sensor_world_pose = link_world_pose.value() * this->sensor_pose_;

  std::vector<gz::math::Vector3d> points;
  points.reserve(this->conductors_.size());

  if (!this->IsWithinCorridorFootprint(sensor_world_pose.Pos()))
  {
    this->PublishPointCloud(info.simTime, points);
    this->last_publish_time_ = info.simTime;
    return;
  }

  for (const auto & conductor : this->conductors_)
  {
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

    points.push_back(noisy_point_sensor);
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
  this->corridor_lateral_axis_world_ =
      gz::math::Vector3d{-span_axis.Y(), span_axis.X(), 0.0};

  this->corridor_min_span_ = std::numeric_limits<double>::infinity();
  this->corridor_max_span_ = -std::numeric_limits<double>::infinity();
  this->corridor_min_lateral_ = std::numeric_limits<double>::infinity();
  this->corridor_max_lateral_ = -std::numeric_limits<double>::infinity();

  for (const auto & conductor : this->conductors_)
  {
    for (const auto & sample : conductor.samples_world)
    {
      const gz::math::Vector3d relative = sample - this->corridor_origin_world_;
      const double span = relative.Dot(this->corridor_span_axis_world_);
      const double lateral = relative.Dot(this->corridor_lateral_axis_world_);
      this->corridor_min_span_ = std::min(this->corridor_min_span_, span);
      this->corridor_max_span_ = std::max(this->corridor_max_span_, span);
      this->corridor_min_lateral_ = std::min(this->corridor_min_lateral_, lateral);
      this->corridor_max_lateral_ = std::max(this->corridor_max_lateral_, lateral);
    }
  }

  this->corridor_bounds_valid_ =
      std::isfinite(this->corridor_min_span_) &&
      std::isfinite(this->corridor_max_span_) &&
      std::isfinite(this->corridor_min_lateral_) &&
      std::isfinite(this->corridor_max_lateral_);
}

bool MmwaveConductorSensorPlugin::IsWithinCorridorFootprint(
    const gz::math::Vector3d & world_point) const
{
  if (!this->corridor_bounds_valid_)
  {
    return true;
  }

  const gz::math::Vector3d relative = world_point - this->corridor_origin_world_;
  const double span = relative.Dot(this->corridor_span_axis_world_);
  const double lateral = relative.Dot(this->corridor_lateral_axis_world_);
  const double margin = std::max(0.0, this->corridor_margin_m_);

  return span >= this->corridor_min_span_ - margin &&
      span <= this->corridor_max_span_ + margin &&
      lateral >= this->corridor_min_lateral_ - margin &&
      lateral <= this->corridor_max_lateral_ + margin;
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
  if (range < this->min_point_dist_ || range > this->max_point_dist_)
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

void MmwaveConductorSensorPlugin::PublishPointCloud(
    const std::chrono::steady_clock::duration & sim_time,
    const std::vector<gz::math::Vector3d> & points)
{
  gz::msgs::PointCloudPacked message;
  gz::msgs::InitPointCloudPacked(
      message,
      this->frame_id_,
      false,
      {{"xyz", gz::msgs::PointCloudPacked::Field::FLOAT32}});

  // Synthetic detections are generated from the latest drone pose. Stamping at
  // zero avoids a TF race where the cloud arrives before the matching odometry
  // transform has reached RViz.
  if (!this->stamp_at_latest_tf_)
  {
    *message.mutable_header()->mutable_stamp() = gz::msgs::Convert(sim_time);
  }
  message.set_height(1);
  message.set_width(static_cast<std::uint32_t>(points.size()));
  message.set_is_dense(true);
  message.set_row_step(message.point_step() * message.width());
  message.mutable_data()->resize(points.size() * message.point_step());

  if (!points.empty())
  {
    gz::msgs::PointCloudPackedIterator<float> x_it(message, "x");
    gz::msgs::PointCloudPackedIterator<float> y_it(message, "y");
    gz::msgs::PointCloudPackedIterator<float> z_it(message, "z");

    for (const auto & point : points)
    {
      *x_it = static_cast<float>(point.X());
      *y_it = static_cast<float>(point.Y());
      *z_it = static_cast<float>(point.Z());
      ++x_it;
      ++y_it;
      ++z_it;
    }
  }

  this->publisher_.Publish(message);
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
