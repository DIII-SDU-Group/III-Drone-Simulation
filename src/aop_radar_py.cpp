// r21 / WO-2026-09-29-001: Python binding of the simulator-v2 AOP radar core (analysis, layout studies, tests).
// The binding calls exactly the same header-only model and YAML loader as the Gazebo plugin.
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <array>
#include <string>
#include <vector>

#include "iii_drone_simulation/aop_radar_accel.hpp"
#include "iii_drone_simulation/aop_radar_config.hpp"
#include "iii_drone_simulation/aop_radar_model.hpp"

namespace py = pybind11;
using namespace aop_radar;

namespace
{

class PyModel
{
public:
  PyModel(
    const std::string & config_path, const std::vector<std::vector<std::array<double, 3>>> & conductors,
    const std::vector<std::array<double, 6>> & voxels, std::uint64_t radar_stream_seed, py::dict overrides,
    const std::vector<std::array<double, 8>> & members)
  {
    YAML::Node root = YAML::LoadFile(config_path);
    for (auto item : overrides)
    {
      // overrides: {"calibration.false_alarms_per_frame": 0.0, "processing.cfar_threshold_db": 18, ...}
      const auto key = py::str(item.first).cast<std::string>();
      const auto dot = key.find('.');
      YAML::Node target = dot == std::string::npos ? root[key] : root[key.substr(0, dot)][key.substr(dot + 1)];
      if (py::isinstance<py::bool_>(item.second)) {target = item.second.cast<bool>();}
      else if (py::isinstance<py::str>(item.second)) {target = item.second.cast<std::string>();}
      else if (py::isinstance<py::int_>(item.second)) {target = item.second.cast<long long>();}
      else {target = item.second.cast<double>();}
    }
    config_ = load_model_config(root);
    std::vector<Conductor> cs;
    for (const auto & poly : conductors)
    {
      Conductor c;
      for (const auto & p : poly) {c.samples_world.emplace_back(p[0], p[1], p[2]);}
      cs.push_back(std::move(c));
    }
    std::vector<VoxelScatterer> vs;
    vs.reserve(voxels.size());
    for (const auto & v : voxels)
    {
      vs.push_back(VoxelScatterer{Vec3{v[0], v[1], v[2]}, v[3], static_cast<std::uint8_t>(v[4]), static_cast<int>(v[5])});
    }
    model_.configure(config_.profile, config_.antenna, config_.calibration, config_.processing, std::move(cs),
      std::move(vs), radar_stream_seed, config_.scene_seed);
    std::vector<MemberSegment> ms;
    for (const auto & m : members)
    {
      ms.push_back(MemberSegment{Vec3{m[0], m[1], m[2]}, Vec3{m[3], m[4], m[5]}, m[6], static_cast<int>(m[7])});
    }
    model_.set_pylon_members(std::move(ms));
    backend_ = attach_backend(model_, config_.backend, &backend_note_);
  }

  std::string backend() const {return backend_;}
  std::string backend_note() const {return backend_note_;}

  py::dict scan(
    const std::array<double, 3> & pos, const std::array<double, 9> & rot, const std::array<double, 3> & vel,
    std::uint64_t seq, double overlap) const
  {
    SensorState s;
    s.position_world = {pos[0], pos[1], pos[2]};
    s.rotation_world_from_sensor.m = rot;
    s.velocity_world = {vel[0], vel[1], vel[2]};
    ScanDiagnostics diag;
    const auto dets = model_.scan(s, seq, overlap, &diag);
    const auto n = static_cast<py::ssize_t>(dets.size());
    py::array_t<float> xyzvsn({n, static_cast<py::ssize_t>(6)});
    py::array_t<int> ints({n, static_cast<py::ssize_t>(5)});      // mechanism, source, n_scatterers, rbin, dbin
    py::array_t<double> dbl({n, static_cast<py::ssize_t>(10)});   // ideal sensor xyz, ideal world xyz, cparam, gain, mean snr, fraction
    auto a = xyzvsn.mutable_unchecked<2>();
    auto b = ints.mutable_unchecked<2>();
    auto c = dbl.mutable_unchecked<2>();
    for (py::ssize_t i = 0; i < n; ++i)
    {
      const auto & d = dets[static_cast<std::size_t>(i)];
      a(i, 0) = d.x; a(i, 1) = d.y; a(i, 2) = d.z; a(i, 3) = d.velocity; a(i, 4) = d.snr_db; a(i, 5) = d.noise_db;
      b(i, 0) = static_cast<int>(d.mechanism); b(i, 1) = d.source_index; b(i, 2) = d.scatterers_in_cell;
      b(i, 3) = d.range_bin; b(i, 4) = d.doppler_bin;
      c(i, 0) = d.ideal_point_sensor.x; c(i, 1) = d.ideal_point_sensor.y; c(i, 2) = d.ideal_point_sensor.z;
      c(i, 3) = d.ideal_point_world.x; c(i, 4) = d.ideal_point_world.y; c(i, 5) = d.ideal_point_world.z;
      c(i, 6) = d.conductor_parameter_m; c(i, 7) = d.two_way_gain_db; c(i, 8) = d.mean_snr_db; c(i, 9) = d.dominant_power_fraction;
    }
    py::dict dd;
    dd["candidates"] = diag.candidates; dd["cells"] = diag.cells; dd["cfar_pass"] = diag.cfar_pass; dd["peaks"] = diag.peaks;
    dd["false_alarms"] = diag.false_alarms; dd["ghosts"] = diag.ghosts; dd["capped"] = diag.capped;
    dd["noise_rise_db"] = diag.noise_rise_db;
    std::vector<std::size_t> bym(diag.by_mechanism.begin(), diag.by_mechanism.end());
    dd["by_mechanism"] = bym;
    py::dict out;
    out["runtime"] = xyzvsn; out["truth_int"] = ints; out["truth_dbl"] = dbl; out["diag"] = dd;
    return out;
  }

  // Diagnostic: every range-Doppler cell with CFAR stage, noise estimates and per-mechanism power.
  py::dict scan_trace(
    const std::array<double, 3> & pos, const std::array<double, 9> & rot, const std::array<double, 3> & vel,
    std::uint64_t seq, double overlap) const
  {
    SensorState s;
    s.position_world = {pos[0], pos[1], pos[2]};
    s.rotation_world_from_sensor.m = rot;
    s.velocity_world = {vel[0], vel[1], vel[2]};
    ScanDiagnostics diag;
    std::vector<CellTrace> trace;
    const auto dets = model_.scan(s, seq, overlap, &diag, &trace);
    const auto n = static_cast<py::ssize_t>(trace.size());
    py::array_t<int> ints({n, static_cast<py::ssize_t>(5)});       // range bin, doppler bin, stage, members, dominant source
    py::array_t<double> dbl({n, static_cast<py::ssize_t>(22)});    // power, coherent, incoherent, noise l/r/doppler, mech_power[8], mech_count[8]
    auto a = ints.mutable_unchecked<2>();
    auto b = dbl.mutable_unchecked<2>();
    for (py::ssize_t i = 0; i < n; ++i)
    {
      const auto & t = trace[static_cast<std::size_t>(i)];
      a(i, 0) = t.range_bin; a(i, 1) = t.doppler_bin; a(i, 2) = t.stage; a(i, 3) = t.members; a(i, 4) = t.dominant_source;
      b(i, 0) = t.power; b(i, 1) = t.coherent_power; b(i, 2) = t.incoherent_power;
      b(i, 3) = t.noise_left; b(i, 4) = t.noise_right; b(i, 5) = t.noise_doppler;
      for (int k = 0; k < 8; ++k) {b(i, 6 + k) = t.mech_power[static_cast<std::size_t>(k)]; b(i, 14 + k) = t.mech_count[static_cast<std::size_t>(k)];}
    }
    py::dict out;
    out["cells_int"] = ints; out["cells_dbl"] = dbl; out["n_detections"] = dets.size();
    out["capped"] = diag.capped;
    std::vector<std::size_t> bym(diag.by_mechanism.begin(), diag.by_mechanism.end());
    out["by_mechanism"] = bym;
    return out;
  }

  // Diagnostic: per-stage wall time (ns) of `repeats` identical scans (outputs identical to scan()).
  py::dict scan_profile(
    const std::array<double, 3> & pos, const std::array<double, 9> & rot, const std::array<double, 3> & vel,
    std::uint64_t seq, double overlap, int repeats) const
  {
    SensorState s;
    s.position_world = {pos[0], pos[1], pos[2]};
    s.rotation_world_from_sensor.m = rot;
    s.velocity_world = {vel[0], vel[1], vel[2]};
    StageProfile prof;
    ScanDiagnostics diag;
    std::size_t n = 0;
    for (int r = 0; r < std::max(1, repeats); ++r) {n = model_.scan(s, seq, overlap, &diag, nullptr, &prof).size();}
    py::dict out;
    out["stage_ns"] = std::vector<std::int64_t>(prof.ns.begin(), prof.ns.end());
    out["repeats"] = std::max(1, repeats);
    out["candidates"] = diag.candidates; out["cells"] = diag.cells; out["detections"] = n;
    return out;
  }

  py::dict profile() const
  {
    const auto & p = model_.profile();
    py::dict d;
    d["lambda_m"] = p.lambda_m; d["range_resolution_m"] = p.range_resolution_m; d["range_bin_m"] = p.range_bin_m;
    d["max_range_m"] = p.max_range_m; d["vmax_mps"] = p.vmax_mps; d["doppler_resolution_mps"] = p.doppler_resolution_mps;
    d["doppler_bin_mps"] = p.doppler_bin_mps; d["range_fft"] = p.range_fft; d["doppler_fft"] = p.doppler_fft;
    d["frame_active_s"] = p.frame_active_s; d["frame_period_ms"] = p.frame_period_ms;
    d["snr_constant_db"] = model_.snr_constant_db(); d["mode"] = config_.mode; d["profile_id"] = config_.profile_id;
    return d;
  }

  double analytic_snr_db(double sigma, double range, double gain_db) const {return model_.analytic_snr_db(sigma, range, gain_db);}
  std::size_t member_count() const {return model_.pylon_member_count();}
  double two_way_gain_db(double az, double el) const {return config_.antenna.two_way_db(az, el);}

private:
  ModelConfig config_;
  AopRadarModel model_;
  std::string backend_{"cpu"}, backend_note_;
};

}  // namespace

PYBIND11_MODULE(aop_radar_py, m)
{
  m.doc() = "r21 simulator-v2 IWR6843AOP post-detection radar core (same code as the Gazebo plugin)";
  py::class_<PyModel>(m, "Model")
  .def(py::init<const std::string &, const std::vector<std::vector<std::array<double, 3>>> &,
    const std::vector<std::array<double, 6>> &, std::uint64_t, py::dict, const std::vector<std::array<double, 8>> &>(),
    py::arg("config_path"), py::arg("conductors"), py::arg("voxels"), py::arg("radar_stream_seed"),
    py::arg("overrides") = py::dict(), py::arg("pylon_members") = std::vector<std::array<double, 8>>())
  .def("scan", &PyModel::scan, py::arg("position"), py::arg("rotation_row_major"), py::arg("velocity"),
    py::arg("scan_sequence"), py::arg("schedule_overlap_fraction") = 0.0)
  .def("scan_trace", &PyModel::scan_trace, py::arg("position"), py::arg("rotation_row_major"), py::arg("velocity"),
    py::arg("scan_sequence"), py::arg("schedule_overlap_fraction") = 0.0)
  .def("scan_profile", &PyModel::scan_profile, py::arg("position"), py::arg("rotation_row_major"), py::arg("velocity"),
    py::arg("scan_sequence"), py::arg("schedule_overlap_fraction") = 0.0, py::arg("repeats") = 1)
  .def("profile", &PyModel::profile)
  .def("pylon_member_count", [](const PyModel & m) {return m.member_count();})
  .def("backend", &PyModel::backend)
  .def("backend_note", &PyModel::backend_note)
  .def("analytic_snr_db", &PyModel::analytic_snr_db, py::arg("sigma_m2"), py::arg("range_m"), py::arg("gain_db") = 0.0)
  .def("two_way_gain_db", &PyModel::two_way_gain_db);
  m.def("stream_seed", [](std::uint64_t sim_seed, std::uint64_t radar_seed) {return mix(sim_seed, radar_seed);},
    "radar noise stream seed exactly as the Gazebo plugin derives it");
  m.attr("MECHANISMS") = py::make_tuple("NONE", "CONDUCTOR_SPECULAR", "CONDUCTOR_DIFFUSE", "PYLON", "TERRAIN",
      "STRUCTURE", "FALSE_ALARM", "MULTIPATH_GHOST");
}
