// r22 / WO-2026-09-29-002 Backlog 03: accelerator backends for the simulator-v2 candidate stage.
//
// * CpuEmulatedAccelerator: the accelerator data layout and shared math evaluated on the host; it must be
//   bitwise identical to the CPU reference path (proves export order, assembly and math mirroring);
// * make_cuda_accelerator(): CUDA backend (aop_radar_cuda.cu, built only with AOP_WITH_CUDA); identical
//   algorithm on the device, differing from the CPU reference only by device libm rounding.
#pragma once

#include <cstddef>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "iii_drone_simulation/aop_radar_accel_common.hpp"
#include "iii_drone_simulation/aop_radar_model.hpp"

namespace aop_radar
{

static_assert(sizeof(aop_accel::Contrib) == sizeof(Contribution), "Contribution layout mismatch");
static_assert(offsetof(aop_accel::Contrib, snr_lin) == offsetof(Contribution, snr_lin), "layout");
static_assert(offsetof(aop_accel::Contrib, range_bin) == offsetof(Contribution, range_bin), "layout");
static_assert(offsetof(aop_accel::Contrib, mech) == offsetof(Contribution, mech), "layout");
static_assert(offsetof(aop_accel::Contrib, source_index) == offsetof(Contribution, source_index), "layout");
static_assert(offsetof(aop_accel::Contrib, conductor_parameter) == offsetof(Contribution, conductor_parameter), "layout");

struct AccelScene
{
  aop_accel::Consts k{};
  std::vector<aop_accel::Seg> segs;
  std::vector<aop_accel::Elem> elems;
  std::vector<std::uint32_t> conductor_begin;     // element offsets per conductor (size conductors + 1)
  std::vector<aop_accel::Vox> vox;
  std::vector<aop_accel::Mem> mem;
};

inline aop_accel::V3 v3(const Vec3 & v) {return {v.x, v.y, v.z};}

inline AccelScene make_accel_scene(const SceneExport & e)
{
  AccelScene a;
  auto & k = a.k;
  const auto db2lin = [](double db) {return std::pow(10.0, db / 10.0);};
  k.lambda = e.profile.lambda_m; k.range_bin_m = e.profile.range_bin_m; k.doppler_bin_mps = e.profile.doppler_bin_mps;
  k.max_range = e.proc.max_range_m; k.rmax = e.proc.max_range_m + e.cal.cull_margin_m; k.snr_constant = e.snr_constant;
  k.range_bias = e.cal.range_bias_m;
  k.diffuse_width = e.cal.conductor_diffuse_aspect_width_deg * kPi / 180.0;
  k.diffuse_per_m = db2lin(e.cal.conductor_diffuse_dbsm_per_m);
  k.terrain_gamma = db2lin(e.cal.terrain_gamma_db);
  k.member_scale = db2lin(e.cal.pylon_member_scale_db);
  k.member_fading_db = e.cal.pylon_member_fading_db;
  k.doppler_fft = e.profile.doppler_fft; k.stream_seed = e.stream_seed;
  const auto n = e.antenna.angles_deg.size();
  if (n > static_cast<std::size_t>(aop_accel::kMaxAngles)) {throw std::runtime_error("antenna table too large for accelerator");}
  k.n_angles = static_cast<int>(n);
  for (std::size_t i = 0; i < n; ++i)
  {
    k.angles[i] = e.antenna.angles_deg[i]; k.tx_az[i] = e.antenna.tx_az[i]; k.rx_az[i] = e.antenna.rx_az[i];
    k.tx_el[i] = e.antenna.tx_el[i]; k.rx_el[i] = e.antenna.rx_el[i];
  }
  for (const auto & s : e.segments) {a.segs.push_back({v3(s.a), v3(s.ab), v3(s.tangent), s.len, s.ds});}
  a.conductor_begin = e.conductor_element_begin;
  for (std::size_t ci = 0; ci + 1 < e.conductor_element_begin.size(); ++ci)
  {
    for (auto i = e.conductor_element_begin[ci]; i < e.conductor_element_begin[ci + 1]; ++i)
    {
      const auto & el = e.elements[i];
      a.elems.push_back({v3(el.p), el.arc, el.phase0, el.segment, static_cast<std::uint32_t>(ci)});
    }
  }
  for (const auto & v : e.voxels) {a.vox.push_back({v3(v.p), v.area, v.sigma_base, v.phase0, v.cls, v.pylon_index});}
  for (const auto & m : e.members) {a.mem.push_back({v3(m.a), v3(m.b), m.radius, m.factor, m.pylon_index});}
  return a;
}

inline aop_accel::Sensor make_sensor(const SensorState & s)
{
  aop_accel::Sensor d{};
  d.pos = v3(s.position_world); d.vel = v3(s.velocity_world);
  for (int i = 0; i < 9; ++i) {d.R[i] = s.rotation_world_from_sensor.m[static_cast<std::size_t>(i)];}
  return d;
}

inline Contribution to_contribution(const aop_accel::Contrib & c)
{
  Contribution out;
  std::memcpy(static_cast<void *>(&out), &c, sizeof(Contribution));
  return out;
}

class CpuEmulatedAccelerator : public CandidateAccelerator
{
public:
  explicit CpuEmulatedAccelerator(const SceneExport & e) : a_(make_accel_scene(e)) {}
  std::string name() const override {return "cpu_emulated";}
  bool evaluate(
    const SensorState & s, std::uint64_t scan_sequence, const std::vector<std::vector<double>> & arcs,
    std::vector<std::vector<Contribution>> & diffuse_out, std::vector<Contribution> & voxel_out,
    std::vector<Contribution> & member_out) override
  {
    const auto d = make_sensor(s);
    const std::size_t nc = a_.conductor_begin.size() - 1;
    diffuse_out.assign(nc, {});
    aop_accel::Contrib c{};
    for (std::size_t ci = 0; ci < nc; ++ci)
    {
      const double * ap = arcs[ci].empty() ? nullptr : arcs[ci].data();
      for (auto i = a_.conductor_begin[ci]; i < a_.conductor_begin[ci + 1]; ++i)
      {
        const auto & el = a_.elems[i];
        if (aop_accel::eval_diffuse(a_.k, d, a_.segs[el.segment], el, ap, static_cast<int>(arcs[ci].size()), c))
        {
          diffuse_out[ci].push_back(to_contribution(c));
        }
      }
    }
    voxel_out.clear();
    for (const auto & v : a_.vox) {if (aop_accel::eval_voxel(a_.k, d, v, c)) {voxel_out.push_back(to_contribution(c));}}
    member_out.clear();
    for (std::size_t i = 0; i < a_.mem.size(); ++i)
    {
      if (aop_accel::eval_member(a_.k, d, a_.mem[i], static_cast<std::uint32_t>(i), scan_sequence, c)) {member_out.push_back(to_contribution(c));}
    }
    return true;
  }

private:
  AccelScene a_;
};

#ifdef AOP_WITH_CUDA
// defined in src/aop_radar_cuda.cu; returns nullptr (and sets *error) when no usable CUDA device exists
std::shared_ptr<CandidateAccelerator> make_cuda_accelerator(const SceneExport & e, std::string * error);
#endif

// Attach the configured backend ("cpu" | "cpu_emulated" | "cuda"); returns the active backend name.
// A failed "cuda" request falls back to the CPU reference path and reports why in *note.
inline std::string attach_backend(AopRadarModel & model, const std::string & requested, std::string * note)
{
  if (requested.empty() || requested == "cpu") {model.set_accelerator(nullptr); return "cpu";}
  if (!model.accelerator_supported())
  {
    if (note) {*note = "accelerator unsupported for this scene (non-precomputable conductor elements); CPU reference used";}
    model.set_accelerator(nullptr);
    return "cpu";
  }
  if (requested == "cpu_emulated")
  {
    model.set_accelerator(std::make_shared<CpuEmulatedAccelerator>(model.export_scene()));
    return "cpu_emulated";
  }
  if (requested == "cuda")
  {
#ifdef AOP_WITH_CUDA
    std::string err;
    auto acc = make_cuda_accelerator(model.export_scene(), &err);
    if (acc) {model.set_accelerator(acc); return "cuda";}
    if (note) {*note = "CUDA backend unavailable (" + err + "); CPU reference used";}
#else
    if (note) {*note = "built without CUDA support; CPU reference used";}
#endif
    model.set_accelerator(nullptr);
    return "cpu";
  }
  throw std::runtime_error("unknown aop radar backend: " + requested);
}

}  // namespace aop_radar
