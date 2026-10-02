// r21 / WO-2026-09-29-001: YAML/JSON loader for the simulator-v2 AOP radar model.
// Shared by the Gazebo plugin and the Python binding so both parse identical configuration.
#pragma once

#include <yaml-cpp/yaml.h>

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include "iii_drone_simulation/aop_radar_model.hpp"

namespace aop_radar
{

template<typename T>
inline void opt(const YAML::Node & n, const char * key, T & value)
{
  if (n && n[key]) {value = n[key].as<T>();}
}

inline std::vector<double> vec(const YAML::Node & n, const char * key)
{
  if (!n || !n[key]) {throw std::runtime_error(std::string("aop radar config missing ") + key);}
  return n[key].as<std::vector<double>>();
}

struct ModelConfig
{
  std::string mode{"AOP_FAST_POINT"};
  std::string profile_id;
  FmcwProfile profile;
  AntennaPattern antenna;
  Calibration calibration;
  Processing processing;
  std::uint64_t scene_seed{0x5CE9E5EEDULL};
  std::string scatterers_path;
  std::string backend{"cpu"};   // r22: candidate-stage backend "cpu" | "cpu_emulated" | "cuda" (processing.backend)
};

inline ModelConfig load_model_config(const YAML::Node & root)
{
  ModelConfig c;
  opt(root, "mode", c.mode);
  opt(root, "profile_id", c.profile_id);
  const auto p = root["profile"];
  if (!p) {throw std::runtime_error("aop radar config missing profile");}
  auto & f = c.profile;
  opt(p, "start_freq_ghz", f.start_freq_ghz); opt(p, "idle_us", f.idle_us); opt(p, "adc_start_us", f.adc_start_us);
  opt(p, "ramp_end_us", f.ramp_end_us); opt(p, "slope_mhz_per_us", f.slope_mhz_per_us);
  opt(p, "sample_rate_ksps", f.sample_rate_ksps); opt(p, "frame_period_ms", f.frame_period_ms);
  opt(p, "num_adc_samples", f.num_adc_samples); opt(p, "num_loops", f.num_loops);
  opt(p, "num_tx", f.num_tx); opt(p, "num_rx", f.num_rx);
  f.derive();
  const auto a = root["antenna"];
  c.antenna.angles_deg = vec(a, "angles_deg");
  c.antenna.tx_az = vec(a, "tx_azimuth"); c.antenna.tx_el = vec(a, "tx_elevation");
  c.antenna.rx_az = vec(a, "rx_azimuth"); c.antenna.rx_el = vec(a, "rx_elevation");
  if (!c.antenna.valid()) {throw std::runtime_error("aop radar antenna tables inconsistent");}
  const auto k = root["calibration"];
  auto & q = c.calibration;
  opt(k, "eirp_dbm", q.eirp_dbm); opt(k, "einf_db", q.einf_db); opt(k, "implementation_loss_db", q.implementation_loss_db);
  opt(k, "snr_offset_db", q.snr_offset_db); opt(k, "conductor_radius_m", q.conductor_radius_m);
  opt(k, "conductor_specular_scale_db", q.conductor_specular_scale_db);
  opt(k, "conductor_specular_fading_db", q.conductor_specular_fading_db);
  opt(k, "conductor_diffuse_dbsm_per_m", q.conductor_diffuse_dbsm_per_m);
  opt(k, "conductor_diffuse_aspect_width_deg", q.conductor_diffuse_aspect_width_deg);
  opt(k, "conductor_diffuse_step_fraction", q.conductor_diffuse_step_fraction);
  opt(k, "pylon_reflectivity_db", q.pylon_reflectivity_db); opt(k, "pylon_lognormal_db", q.pylon_lognormal_db);
  opt(k, "structure_reflectivity_db", q.structure_reflectivity_db); opt(k, "structure_lognormal_db", q.structure_lognormal_db);
  opt(k, "terrain_gamma_db", q.terrain_gamma_db); opt(k, "false_alarms_per_frame", q.false_alarms_per_frame);
  opt(k, "multipath_ground_reflection_db", q.multipath_ground_reflection_db); opt(k, "multipath_enabled", q.multipath_enabled);
  opt(k, "angle_floor_deg", q.angle_floor_deg); opt(k, "range_bias_m", q.range_bias_m);
  opt(k, "noise_reference_db", q.noise_reference_db); opt(k, "interference_inr_db", q.interference_inr_db);
  opt(k, "terrain_z_m", q.terrain_z_m); opt(k, "cull_margin_m", q.cull_margin_m);
  opt(k, "pylon_member_scale_db", q.pylon_member_scale_db); opt(k, "pylon_member_lognormal_db", q.pylon_member_lognormal_db);
  opt(k, "pylon_member_fading_db", q.pylon_member_fading_db);
  const auto r = root["processing"];
  auto & g = c.processing;
  opt(r, "cfar_threshold_db", g.cfar_threshold_db); opt(r, "cfar_range_window", g.cfar_range_window);
  opt(r, "cfar_range_guard", g.cfar_range_guard); opt(r, "cfar_doppler_window", g.cfar_doppler_window);
  opt(r, "cfar_doppler_guard", g.cfar_doppler_guard); opt(r, "peak_grouping", g.peak_grouping);
  opt(r, "multi_object_beamforming", g.multi_object_beamforming); opt(r, "min_range_m", g.min_range_m);
  opt(r, "max_range_m", g.max_range_m);
  if (r && r["max_points"]) {g.max_points = r["max_points"].as<std::size_t>();}
  if (r && r["backend"]) {c.backend = r["backend"].as<std::string>();}
  if (r && r["cfar_domain"])
  {
    const auto d = r["cfar_domain"].as<std::string>();
    if (d != "linear" && d != "log") {throw std::runtime_error("aop radar config: cfar_domain must be linear or log");}
    g.cfar_log_domain = d == "log";
  }
  if (root["scene_seed"]) {c.scene_seed = root["scene_seed"].as<std::uint64_t>();}
  opt(root, "scatterers_path", c.scatterers_path);
  return c;
}

inline ModelConfig load_model_config_file(const std::string & path) {return load_model_config(YAML::LoadFile(path));}

inline std::vector<Conductor> load_conductors_file(const std::string & path)
{
  std::vector<Conductor> out;
  const YAML::Node root = YAML::LoadFile(path);
  for (const auto & cn : root["conductors"])
  {
    Conductor c;
    for (const auto & s : cn["samples"]) {c.samples_world.emplace_back(s[0].as<double>(), s[1].as<double>(), s[2].as<double>());}
    if (c.samples_world.size() >= 2) {out.push_back(std::move(c));}
  }
  return out;
}

// scene_scatterers_v1.json: {"scatterers": [[x, y, z, area, class, pylon_index], ...]}
inline std::vector<VoxelScatterer> load_scatterers_file(const std::string & path)
{
  std::vector<VoxelScatterer> out;
  if (path.empty()) {return out;}
  const YAML::Node root = YAML::LoadFile(path);
  const auto rows = root["scatterers"];
  out.reserve(rows.size());
  for (const auto & row : rows)
  {
    out.push_back(VoxelScatterer{Vec3{row[0].as<double>(), row[1].as<double>(), row[2].as<double>()}, row[3].as<double>(),
        static_cast<std::uint8_t>(row[4].as<int>()), row[5].as<int>()});
  }
  return out;
}

// r22 scene file extension: {"pylon_members": [[x0, y0, z0, x1, y1, z1, radius_m, pylon_index], ...]} (optional)
inline std::vector<MemberSegment> load_pylon_members_file(const std::string & path)
{
  std::vector<MemberSegment> out;
  if (path.empty()) {return out;}
  const YAML::Node root = YAML::LoadFile(path);
  const auto rows = root["pylon_members"];
  if (!rows) {return out;}
  out.reserve(rows.size());
  for (const auto & row : rows)
  {
    out.push_back(MemberSegment{Vec3{row[0].as<double>(), row[1].as<double>(), row[2].as<double>()},
        Vec3{row[3].as<double>(), row[4].as<double>(), row[5].as<double>()}, row[6].as<double>(), row[7].as<int>()});
  }
  return out;
}

}  // namespace aop_radar
