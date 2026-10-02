// r21 / WO-2026-09-29-001: IWR6843AOP-like post-detection radar model (simulator-v2 core).
//
// Header-only, dependency-free C++17.  The same core serves Radar U and Radar F
// (configuration, not duplicated logic), the Gazebo plugin and the Python binding.
//
// Pipeline per scan (Backlog 03/04 architecture):
//   1 physical scene geometry      conductors (polylines), pylon/structure/terrain voxels
//   2 hardware profile             AOP two-way gain tables (SWRS237C Fig 7-2/7-3), EIRP, EINF
//   3 FMCW profile                 range/Doppler bins, max range, vmax, processing gain
//   4 candidate scattering         conductor specular glints + aspect-dependent diffuse elements,
//                                  pylon members, terrain (constant-gamma), structures, ground-bounce ghosts
//   5 antenna gain / visibility    continuous gain; no detection outside the characterised +-80 deg
//   6 SNR / detection              radar equation -> coherent sum per range-Doppler cell with geometric
//                                  phase -> thermal/interference noise -> CFAR (range CASO + Doppler CA)
//                                  -> peak grouping -> multi-object beamforming
//   7 measurement generation       range at bin centre, Doppler at FFT bin (aliased), angle noise vs SNR
//   8 point cloud                  runtime fields x,y,z,velocity,snr,noise only
//   9 evaluator truth              mechanism, source index, ideal point, bins, gains (never runtime)
//
// Determinism: all randomness comes from counter-based streams keyed by
// (radar stream seed, scan sequence) or (scene seed, scatterer index); no shared RNG.
#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace aop_radar
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kC = 299792458.0;
constexpr double kBoltzmannT0 = 1.380649e-23 * 290.0;  // J

struct Vec3
{
  double x{0.0}, y{0.0}, z{0.0};
  Vec3() = default;
  Vec3(double a, double b, double c) : x(a), y(b), z(c) {}
  Vec3 operator+(const Vec3 & o) const {return {x + o.x, y + o.y, z + o.z};}
  Vec3 operator-(const Vec3 & o) const {return {x - o.x, y - o.y, z - o.z};}
  Vec3 operator*(double s) const {return {x * s, y * s, z * s};}
  double dot(const Vec3 & o) const {return x * o.x + y * o.y + z * o.z;}
  Vec3 cross(const Vec3 & o) const {return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x};}
  double norm() const {return std::sqrt(dot(*this));}
};

// Row-major rotation; R * v maps sensor-frame vectors to world.
struct Mat3
{
  std::array<double, 9> m{1, 0, 0, 0, 1, 0, 0, 0, 1};
  Vec3 apply(const Vec3 & v) const
  {
    return {m[0] * v.x + m[1] * v.y + m[2] * v.z, m[3] * v.x + m[4] * v.y + m[5] * v.z,
      m[6] * v.x + m[7] * v.y + m[8] * v.z};
  }
  Vec3 apply_transpose(const Vec3 & v) const
  {
    return {m[0] * v.x + m[3] * v.y + m[6] * v.z, m[1] * v.x + m[4] * v.y + m[7] * v.z,
      m[2] * v.x + m[5] * v.y + m[8] * v.z};
  }
};

// ---------------------------------------------------------------- randomness
inline std::uint64_t splitmix64(std::uint64_t x)
{
  x += 0x9E3779B97F4A7C15ULL;
  x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ULL;
  x = (x ^ (x >> 27)) * 0x94D049BB133111EBULL;
  return x ^ (x >> 31);
}
inline std::uint64_t mix(std::uint64_t a, std::uint64_t b) {return splitmix64(a ^ splitmix64(b + 0x632BE59BD9B4E019ULL));}

class Rng  // xoshiro256**; portable, no std::distribution implementation dependence
{
public:
  explicit Rng(std::uint64_t seed)
  {
    for (auto & v : s_) {seed = splitmix64(seed); v = seed;}
  }
  std::uint64_t next()
  {
    const std::uint64_t result = rotl(s_[1] * 5, 7) * 9;
    const std::uint64_t t = s_[1] << 17;
    s_[2] ^= s_[0]; s_[3] ^= s_[1]; s_[1] ^= s_[2]; s_[0] ^= s_[3]; s_[2] ^= t; s_[3] = rotl(s_[3], 45);
    return result;
  }
  double uniform() {return (next() >> 11) * (1.0 / 9007199254740992.0);}
  double normal()
  {
    if (has_spare_) {has_spare_ = false; return spare_;}
    double u1 = uniform();
    while (u1 <= 1e-300) {u1 = uniform();}
    const double u2 = uniform();
    const double r = std::sqrt(-2.0 * std::log(u1));
    spare_ = r * std::sin(2.0 * kPi * u2);
    has_spare_ = true;
    return r * std::cos(2.0 * kPi * u2);
  }
  double exponential() {double u = uniform(); while (u <= 1e-300) {u = uniform();} return -std::log(u);}
  std::uint32_t poisson(double lambda)
  {
    if (lambda <= 0.0) {return 0u;}
    const double l = std::exp(-lambda);
    std::uint32_t k = 0; double p = 1.0;
    do {++k; p *= uniform();} while (p > l && k < 10000u);
    return k - 1u;
  }

private:
  static std::uint64_t rotl(std::uint64_t x, int k) {return (x << k) | (x >> (64 - k));}
  std::array<std::uint64_t, 4> s_{};
  bool has_spare_{false};
  double spare_{0.0};
};

// ---------------------------------------------------------------- configuration
struct FmcwProfile
{
  double start_freq_ghz{60.0}, idle_us{7.0}, adc_start_us{6.0}, ramp_end_us{49.96};
  double slope_mhz_per_us{80.0}, sample_rate_ksps{12500.0}, frame_period_ms{33.333};
  int num_adc_samples{512}, num_loops{24}, num_tx{3}, num_rx{4};

  // derived (call derive())
  double lambda_m{0}, t_sample_s{0}, range_resolution_m{0}, range_bin_m{0}, max_range_m{0};
  double tdm_period_s{0}, vmax_mps{0}, doppler_resolution_mps{0}, doppler_bin_mps{0}, frame_active_s{0};
  int range_fft{0}, doppler_fft{0};

  static int nextpow2(int n) {int p = 1; while (p < n) {p <<= 1;} return p;}
  void derive()
  {
    const double fs = sample_rate_ksps * 1e3, slope = slope_mhz_per_us * 1e12;
    lambda_m = kC / (start_freq_ghz * 1e9);
    t_sample_s = num_adc_samples / fs;
    range_resolution_m = kC / (2.0 * slope * t_sample_s);
    range_fft = nextpow2(num_adc_samples);
    doppler_fft = nextpow2(num_loops);
    range_bin_m = kC * fs / (2.0 * slope * range_fft);
    max_range_m = 0.8 * fs * kC / (2.0 * slope);   // TI visualizer convention (validated in Backlog 01)
    tdm_period_s = num_tx * (idle_us + ramp_end_us) * 1e-6;
    vmax_mps = lambda_m / (4.0 * tdm_period_s);
    doppler_resolution_mps = lambda_m / (2.0 * num_loops * tdm_period_s);
    doppler_bin_mps = lambda_m / (2.0 * doppler_fft * tdm_period_s);
    frame_active_s = num_tx * num_loops * (idle_us + ramp_end_us) * 1e-6;
  }
  double coherent_integration() const {return static_cast<double>(num_loops) * num_tx * num_rx;}
};

struct AntennaPattern
{
  // One-way normalised gains (dB) on a common angle grid; separable two-way model.
  std::vector<double> angles_deg, tx_az, tx_el, rx_az, rx_el;
  bool valid() const
  {
    const auto n = angles_deg.size();
    return n >= 2 && tx_az.size() == n && tx_el.size() == n && rx_az.size() == n && rx_el.size() == n;
  }
  static double interp(const std::vector<double> & xs, const std::vector<double> & ys, double x)
  {
    // r22 D03-1: NaN-safe (a NaN angle -- scatterer behind the sensor -- previously fell through to the +80 deg entry)
    if (!(x >= xs.front() && x <= xs.back())) {return std::numeric_limits<double>::quiet_NaN();}
    auto it = std::upper_bound(xs.begin(), xs.end(), x);
    if (it == xs.end()) {return ys.back();}
    const auto i = static_cast<std::size_t>(it - xs.begin());
    const double t = (x - xs[i - 1]) / (xs[i] - xs[i - 1]);
    return ys[i - 1] + t * (ys[i] - ys[i - 1]);
  }
  // NaN outside the characterised pattern -> caller treats as undetectable (conservative).
  double two_way_db(double az_deg, double el_deg) const
  {
    return interp(angles_deg, tx_az, az_deg) + interp(angles_deg, rx_az, az_deg) +
           interp(angles_deg, tx_el, el_deg) + interp(angles_deg, rx_el, el_deg);
  }
  double one_way_rx_db(double az_deg, double el_deg) const
  {
    return interp(angles_deg, rx_az, az_deg) + interp(angles_deg, rx_el, el_deg);
  }
};

// Every uncertain realism component is a named calibration parameter (Backlog 04).
struct Calibration
{
  double eirp_dbm{16.0};                 // SWRS237C single-TX EIRP (typ)
  double einf_db{9.0};                   // SWRS237C effective isotropic noise figure (typ)
  double implementation_loss_db{10.0};   // windowing/processing/unmodelled losses; NEEDS_REAL_DATA_CALIBRATION
  double snr_offset_db{0.0};             // global SNR calibration offset; NEEDS_REAL_DATA_CALIBRATION
  double conductor_radius_m{0.0125};
  double conductor_specular_scale_db{0.0};
  double conductor_specular_fading_db{1.0};
  double conductor_diffuse_dbsm_per_m{-25.0};
  double conductor_diffuse_aspect_width_deg{30.0};
  double conductor_diffuse_step_fraction{0.25};  // element length as a fraction of the range bin
  double pylon_reflectivity_db{-3.0};           // sigma per m^2 of member surface
  double pylon_lognormal_db{4.0};
  double structure_reflectivity_db{-12.0};
  double structure_lognormal_db{5.0};
  double terrain_gamma_db{-12.0};               // constant-gamma sigma0 = gamma * sin(grazing)
  double false_alarms_per_frame{0.5};
  double multipath_ground_reflection_db{-10.0};
  bool multipath_enabled{true};
  double angle_floor_deg{1.5};
  double range_bias_m{0.0};
  double noise_reference_db{40.0};              // reported noise field scale (legacy-compatible)
  double interference_inr_db{25.0};             // VIOLATED_SCHEDULE interference-to-noise ratio
  double terrain_z_m{-0.9524341};
  double cull_margin_m{1.0};
  // r22 pylon member specular model (active only when member segments are loaded); NEEDS_REAL_DATA_CALIBRATION
  double pylon_member_scale_db{0.0};            // scale on the thin-cylinder broadside RCS min(pi a R, 2 pi a L^2 / lambda)
  double pylon_member_lognormal_db{3.0};        // static per-member spread (angle-iron vs cylinder cross-section)
  double pylon_member_fading_db{1.0};           // per-scan glint fading (as conductor specular)
};

struct Processing
{
  double cfar_threshold_db{15.0};
  int cfar_range_window{8}, cfar_range_guard{4};
  int cfar_doppler_window{4}, cfar_doppler_guard{2};
  bool peak_grouping{true};
  double multi_object_beamforming{0.5};  // <=0 disables
  double min_range_m{0.25};
  double max_range_m{0.0};               // 0 -> profile max range
  std::size_t max_points{500};
  // r22: CFAR noise averaging domain. "linear" (r21 default) averages window powers; "log" averages
  // log-power as the TI OOB demo does on its log2-magnitude detection matrix (cfarCfg thresholds in dB).
  bool cfar_log_domain{false};
};

enum class Mechanism : std::uint8_t
{
  NONE = 0, CONDUCTOR_SPECULAR = 1, CONDUCTOR_DIFFUSE = 2, PYLON = 3, TERRAIN = 4, STRUCTURE = 5,
  FALSE_ALARM = 6, MULTIPATH_GHOST = 7
};

struct Conductor {std::vector<Vec3> samples_world;};
// r22: straight pylon lattice member (axis segment a->b, effective cylinder radius) for specular glints.
struct MemberSegment {Vec3 a, b; double radius_m; int pylon_index;};
struct VoxelScatterer {Vec3 p; double area_m2; std::uint8_t cls; int pylon_index;};  // cls 0 pylon 1 structure 2 terrain

struct SensorState
{
  Vec3 position_world;
  Mat3 rotation_world_from_sensor;
  Vec3 velocity_world;            // velocity of the sensor origin (incl. lever-arm rotation)
};

struct Detection
{
  // runtime-observable
  float x{0}, y{0}, z{0}, velocity{0}, snr_db{0}, noise_db{0};
  // evaluator-only
  Mechanism mechanism{Mechanism::NONE};
  int source_index{-1};           // conductor index / pylon index / -1
  double dominant_power_fraction{0.0};
  std::uint16_t scatterers_in_cell{0};
  Vec3 ideal_point_sensor, ideal_point_world;
  double conductor_parameter_m{std::numeric_limits<double>::quiet_NaN()};
  double two_way_gain_db{0.0}, mean_snr_db{0.0};
  int range_bin{0}, doppler_bin{0};
};

// Optional evaluator/diagnostic trace of every range-Doppler cell (r22 Backlog 02/03). Filling it never
// changes the detections; it is requested only by analysis tools (nullptr in the plugin hot path).
struct CellTrace
{
  int range_bin{0}, doppler_bin{0};
  double power{0.0}, coherent_power{0.0}, incoherent_power{0.0};
  double noise_left{0.0}, noise_right{0.0}, noise_doppler{0.0};
  std::uint8_t stage{0};            // 0 out of range window, 1 range-CFAR reject, 2 Doppler-CFAR reject, 3 peak-grouping reject, 4 peak
  std::uint16_t members{0};
  std::array<double, 8> mech_power{};   // incoherent mean power per mechanism
  std::array<std::uint16_t, 8> mech_count{};
  int dominant_source{-1};          // source index of the strongest member
};

// Optional wall-clock stage profile of one scan (r22 Backlog 03); nanoseconds per stage, never changes outputs.
struct StageProfile
{
  static constexpr int kStages = 9;
  // 0 conductors (specular+diffuse+ghost), 1 voxels (pylon/terrain/structure lookup+scatter), 2 pylon members,
  // 3 range-Doppler cell accumulation, 4 noise, 5 CFAR (range+Doppler) + peak grouping, 6 angle clustering/emission,
  // 7 false alarms, 8 cap/sort
  std::array<std::int64_t, kStages> ns{};
};

struct ScanDiagnostics
{
  std::size_t candidates{0}, cells{0}, cfar_pass{0}, peaks{0}, false_alarms{0}, ghosts{0}, capped{0};
  double noise_rise_db{0.0};
  std::array<std::size_t, 8> by_mechanism{};
};

// One scatterer contribution to a range-Doppler cell (plain data; shared with accelerator backends).
struct Contribution
{
  Vec3 p_sensor;        // ideal scatterer point, sensor frame
  double snr_lin;       // mean SNR (noise-normalised power)
  double phase;
  double gain_db;
  int range_bin, doppler_bin;
  Mechanism mech;
  int source_index;
  double conductor_parameter;
};

// r22 Backlog 03: static scene/model data exported to an accelerator backend.
struct SceneExport
{
  FmcwProfile profile;
  AntennaPattern antenna;
  Calibration cal;
  Processing proc;
  double snr_constant{0.0};
  std::uint64_t stream_seed{0}, scene_seed{0};
  // conductor diffuse elements, per conductor in the CPU evaluation order
  struct Segment {Vec3 a, ab, tangent; double len, ds; std::uint32_t first, n;};
  struct Element {Vec3 p; double arc, phase0; std::uint32_t segment;};
  std::vector<std::uint32_t> conductor_element_begin;   // size conductors + 1
  std::vector<Segment> segments;
  std::vector<Element> elements;
  // voxels in the CPU bucket-iteration order (ix, iy, index)
  struct Voxel {Vec3 p; double area, sigma_base, phase0; int cls, pylon_index;};
  std::vector<Voxel> voxels;
  // pylon members
  struct Member {Vec3 a, b; double radius, factor; int pylon_index;};
  std::vector<Member> members;
};

// Optional accelerator: evaluates conductor diffuse elements, voxels and pylon members for one scan and returns
// the valid contributions in exactly the CPU order (diffuse per conductor, then voxels, then members).
class CandidateAccelerator
{
public:
  virtual ~CandidateAccelerator() = default;
  virtual bool evaluate(
    const SensorState & s, std::uint64_t scan_sequence, const std::vector<std::vector<double>> & specular_arcs,
    std::vector<std::vector<Contribution>> & diffuse_out, std::vector<Contribution> & voxel_out,
    std::vector<Contribution> & member_out) = 0;
  virtual std::string name() const = 0;
};

// ---------------------------------------------------------------- model
class AopRadarModel
{
public:
  void configure(
    const FmcwProfile & profile, const AntennaPattern & antenna, const Calibration & cal,
    const Processing & proc, std::vector<Conductor> conductors, std::vector<VoxelScatterer> voxels,
    std::uint64_t radar_stream_seed, std::uint64_t scene_seed)
  {
    profile_ = profile; profile_.derive();
    antenna_ = antenna; cal_ = cal; proc_ = proc;
    if (proc_.max_range_m <= 0.0) {proc_.max_range_m = profile_.max_range_m;}
    conductors_ = std::move(conductors);
    voxels_ = std::move(voxels);
    stream_seed_ = radar_stream_seed; scene_seed_ = scene_seed;
    // radar-equation constant (linear): EIRP * lambda^2 * T_s * M / ((4 pi)^3 k T0 EINF L)
    const double eirp_w = std::pow(10.0, (cal_.eirp_dbm - 30.0) / 10.0);
    snr_constant_ = eirp_w * profile_.lambda_m * profile_.lambda_m * profile_.t_sample_s *
      profile_.coherent_integration() / (std::pow(4.0 * kPi, 3) * kBoltzmannT0 *
      db2lin(cal_.einf_db) * db2lin(cal_.implementation_loss_db)) * db2lin(cal_.snr_offset_db);
    build_buckets();
    precompute_static();
    configured_ = antenna_.valid();
  }

  // r22: pylon member segments (optional; empty -> r21 behaviour). Call after configure().
  void set_pylon_members(std::vector<MemberSegment> members)
  {
    members_ = std::move(members);
    member_factor_.resize(members_.size());
    for (std::size_t i = 0; i < members_.size(); ++i)
    {
      const double u1 = std::max(1e-12, scene_uniform(0x5000u, i)), u2 = scene_uniform(0x6000u, i);
      member_factor_[i] = db2lin(cal_.pylon_member_lognormal_db * std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * kPi * u2));
    }
  }
  std::size_t pylon_member_count() const {return members_.size();}

  // r22 Backlog 03: accelerator backend (nullptr -> CPU reference path)
  void set_accelerator(std::shared_ptr<CandidateAccelerator> a) {accel_ = std::move(a);}
  const CandidateAccelerator * accelerator() const {return accel_.get();}
  bool accelerator_supported() const {return diffuse_static_valid_;}

  SceneExport export_scene() const
  {
    SceneExport e;
    e.profile = profile_; e.antenna = antenna_; e.cal = cal_; e.proc = proc_;
    e.snr_constant = snr_constant_; e.stream_seed = stream_seed_; e.scene_seed = scene_seed_;
    e.conductor_element_begin.push_back(0);
    for (std::size_t ci = 0; ci < diffuse_segments_.size(); ++ci)
    {
      for (const auto & sg : diffuse_segments_[ci])
      {
        const auto seg = static_cast<std::uint32_t>(e.segments.size());
        e.segments.push_back({sg.a, sg.ab, sg.tangent, sg.len, sg.ds, static_cast<std::uint32_t>(e.elements.size()), sg.n});
        for (std::uint32_t k = sg.first; k < sg.first + sg.n; ++k)
        {
          e.elements.push_back({diffuse_elements_[k].p, diffuse_elements_[k].arc, diffuse_elements_[k].phase0, seg});
        }
      }
      e.conductor_element_begin.push_back(static_cast<std::uint32_t>(e.elements.size()));
    }
    std::vector<std::uint32_t> order(voxels_.size());
    for (std::uint32_t i = 0; i < order.size(); ++i) {order[i] = i;}
    auto bx = [&](std::uint32_t i) {return static_cast<std::int64_t>(std::floor(voxels_[i].p.x / kBucket));};
    auto by = [&](std::uint32_t i) {return static_cast<std::int64_t>(std::floor(voxels_[i].p.y / kBucket));};
    std::sort(order.begin(), order.end(), [&](std::uint32_t a, std::uint32_t b) {
        const auto ax = bx(a), bxx = bx(b);
        if (ax != bxx) {return ax < bxx;}
        const auto ay = by(a), byy = by(b);
        return ay != byy ? ay < byy : a < b;});
    for (const auto i : order)
    {
      const auto & v = voxels_[i];
      e.voxels.push_back({v.p, v.area_m2, voxel_sigma_[i], voxel_phase0_[i], v.cls, v.pylon_index});
    }
    for (std::size_t i = 0; i < members_.size(); ++i)
    {
      e.members.push_back({members_[i].a, members_[i].b, members_[i].radius_m, member_factor_[i], members_[i].pylon_index});
    }
    return e;
  }

  bool configured() const {return configured_;}
  const FmcwProfile & profile() const {return profile_;}
  double snr_constant_db() const {return 10.0 * std::log10(snr_constant_);}

  // Mean SNR (dB) of a point target of RCS sigma (m^2) at range r on boresight (analytic helper).
  double analytic_snr_db(double sigma_m2, double range_m, double gain_db = 0.0) const
  {
    return 10.0 * std::log10(snr_constant_ * sigma_m2 / std::pow(range_m, 4)) + gain_db;
  }

  std::vector<Detection> scan(
    const SensorState & s, std::uint64_t scan_sequence, double schedule_overlap_fraction,
    ScanDiagnostics * diag_out = nullptr, std::vector<CellTrace> * trace_out = nullptr,
    StageProfile * prof_out = nullptr) const
  {
    using clk = std::chrono::steady_clock;
    auto t_last = prof_out ? clk::now() : clk::time_point{};
    auto lap = [&](int stage) {
        if (!prof_out) {return;}
        const auto t = clk::now();
        prof_out->ns[static_cast<std::size_t>(stage)] += std::chrono::duration_cast<std::chrono::nanoseconds>(t - t_last).count();
        t_last = t;
      };
    ScanDiagnostics diag;
    std::vector<Detection> out;
    if (!configured_) {if (diag_out) {*diag_out = diag;} return out;}
    Rng rng(mix(stream_seed_, scan_sequence));
    const double noise_power = 1.0 + std::clamp(schedule_overlap_fraction, 0.0, 1.0) * db2lin(cal_.interference_inr_db);
    diag.noise_rise_db = 10.0 * std::log10(noise_power);

    std::vector<Contribution> contribs;
    contribs.reserve(32768);
    if (accel_ && diffuse_static_valid_ && accelerated_candidates(s, scan_sequence, rng, contribs, diag, lap))
    {
      // accelerator produced the complete candidate list in CPU order
    }
    else
    {
      contribs.clear();
      add_conductors(s, rng, contribs, diag);
      lap(0);
      add_voxels(s, contribs);
      lap(1);
      add_members(s, scan_sequence, contribs);
      lap(2);
    }
    diag.candidates = contribs.size();

    // --- range-Doppler cells: coherent sum with geometric phase (r22 Backlog 03: dense cell grid + CSR member
    // lists; cells are still created in contribution order, so sums, noise draws and outputs are unchanged)
    const int nd_grid = profile_.doppler_fft;
    std::vector<std::int32_t> grid(static_cast<std::size_t>(grid_rows_) * static_cast<std::size_t>(nd_grid), -1);
    std::unordered_map<std::int64_t, std::size_t> overflow;   // bins outside the dense grid (not expected)
    auto grid_slot = [&](int rb, int db) -> std::int32_t * {
        const int row = rb + kGridPad;
        const int col = db + nd_grid / 2;
        if (row < 0 || row >= grid_rows_ || col < 0 || col >= nd_grid) {return nullptr;}
        return &grid[static_cast<std::size_t>(row) * static_cast<std::size_t>(nd_grid) + static_cast<std::size_t>(col)];
      };
    auto find_cell = [&](int rb, int db) -> std::int64_t {
        const auto * slot = grid_slot(rb, db);
        if (slot) {return *slot;}
        auto it = overflow.find(cell_key(rb, db));
        return it == overflow.end() ? -1 : static_cast<std::int64_t>(it->second);
      };
    std::vector<Cell> cells;
    cells.reserve(4096);
    std::vector<std::uint32_t> contrib_cell(contribs.size());
    for (std::size_t i = 0; i < contribs.size(); ++i)
    {
      const auto & c = contribs[i];
      auto * slot = grid_slot(c.range_bin, c.doppler_bin);
      std::size_t ci;
      if (slot)
      {
        if (*slot < 0) {*slot = static_cast<std::int32_t>(cells.size()); Cell fresh; fresh.range_bin = c.range_bin; fresh.doppler_bin = c.doppler_bin; cells.push_back(fresh);}
        ci = static_cast<std::size_t>(*slot);
      }
      else
      {
        auto it = overflow.find(cell_key(c.range_bin, c.doppler_bin));
        if (it == overflow.end())
        {
          ci = cells.size(); overflow.emplace(cell_key(c.range_bin, c.doppler_bin), ci);
          Cell fresh; fresh.range_bin = c.range_bin; fresh.doppler_bin = c.doppler_bin; cells.push_back(fresh);
        }
        else {ci = it->second;}
      }
      Cell & cell = cells[ci];
      const double amp = std::sqrt(c.snr_lin);
      cell.re += amp * std::cos(c.phase);
      cell.im += amp * std::sin(c.phase);
      ++cell.member_count;
      contrib_cell[i] = static_cast<std::uint32_t>(ci);
    }
    // CSR member lists in contribution order
    std::vector<std::uint32_t> members_flat(contribs.size());
    {
      std::uint32_t off = 0;
      for (auto & cell : cells) {cell.member_begin = off; off += cell.member_count; cell.member_count = 0;}
      for (std::size_t i = 0; i < contribs.size(); ++i)
      {
        Cell & cell = cells[contrib_cell[i]];
        members_flat[cell.member_begin + cell.member_count++] = static_cast<std::uint32_t>(i);
      }
    }
    diag.cells = cells.size();
    lap(3);
    const double sig = std::sqrt(noise_power / 2.0);
    for (auto & cell : cells)
    {
      const double re = cell.re + sig * rng.normal();
      const double im = cell.im + sig * rng.normal();
      cell.power = re * re + im * im;
    }
    auto power_at = [&](int rb, int db) -> double {
        const auto ci = find_cell(rb, db);
        return ci < 0 ? noise_power : cells[static_cast<std::size_t>(ci)].power;
      };
    // CFAR training statistic of one window cell: linear power, or (log domain) natural-log power where an
    // empty (noise-only) cell contributes the expected log of exponential noise, ln(noise) - Euler gamma.
    const double empty_log = std::log(noise_power) - 0.5772156649015329;
    auto train_at = [&](int rb, int db) -> double {
        if (!proc_.cfar_log_domain) {return power_at(rb, db);}
        const auto ci = find_cell(rb, db);
        return ci < 0 ? empty_log : std::log(std::max(1e-300, cells[static_cast<std::size_t>(ci)].power));
      };
    auto window_noise = [&](double sum, int n) -> double {
        return proc_.cfar_log_domain ? std::exp(sum / n) : sum / n;
      };
    lap(4);
    const double threshold = db2lin(proc_.cfar_threshold_db);
    const int nd = profile_.doppler_fft;
    const int min_bin = static_cast<int>(std::ceil(proc_.min_range_m / profile_.range_bin_m));
    const int max_bin = static_cast<int>(std::floor(proc_.max_range_m / profile_.range_bin_m));

    struct Peak {std::size_t cell; double noise_est;};
    std::vector<Peak> peaks;
    if (trace_out) {trace_out->clear(); trace_out->reserve(cells.size());}
    for (std::size_t ci = 0; ci < cells.size(); ++ci)
    {
      const Cell & cell = cells[ci];
      CellTrace * tr = nullptr;
      if (trace_out) {trace_out->push_back(trace_cell(cell, contribs, members_flat)); tr = &trace_out->back();}
      if (cell.range_bin < min_bin || cell.range_bin > max_bin) {continue;}
      // range CFAR, CASO
      double left = 0.0, right = 0.0;
      for (int k = proc_.cfar_range_guard + 1; k <= proc_.cfar_range_guard + proc_.cfar_range_window; ++k)
      {
        left += train_at(cell.range_bin - k, cell.doppler_bin);
        right += train_at(cell.range_bin + k, cell.doppler_bin);
      }
      const double noise_range = window_noise(std::min(left, right), proc_.cfar_range_window);
      if (tr) {
        tr->noise_left = window_noise(left, proc_.cfar_range_window);
        tr->noise_right = window_noise(right, proc_.cfar_range_window); tr->stage = 1;
      }
      if (cell.power <= threshold * noise_range) {continue;}
      // Doppler CFAR, CA, cyclic
      double dsum = 0.0;
      for (int k = proc_.cfar_doppler_guard + 1; k <= proc_.cfar_doppler_guard + proc_.cfar_doppler_window; ++k)
      {
        dsum += train_at(cell.range_bin, wrap_doppler(cell.doppler_bin - k, nd));
        dsum += train_at(cell.range_bin, wrap_doppler(cell.doppler_bin + k, nd));
      }
      const double noise_doppler = window_noise(dsum, 2 * proc_.cfar_doppler_window);
      if (tr) {tr->noise_doppler = noise_doppler; tr->stage = 2;}
      if (cell.power <= threshold * noise_doppler) {continue;}
      ++diag.cfar_pass;
      if (tr) {tr->stage = 3;}
      if (proc_.peak_grouping)
      {
        if (cell.power < power_at(cell.range_bin - 1, cell.doppler_bin) ||
            cell.power < power_at(cell.range_bin + 1, cell.doppler_bin) ||
            cell.power < power_at(cell.range_bin, wrap_doppler(cell.doppler_bin - 1, nd)) ||
            cell.power < power_at(cell.range_bin, wrap_doppler(cell.doppler_bin + 1, nd)))
        {
          continue;
        }
      }
      if (tr) {tr->stage = 4;}
      peaks.push_back({ci, noise_range});
    }
    diag.peaks = peaks.size();
    lap(5);

    const double beamwidth_rad = 1.77 / 4.0;  // ~ lambda/(N d), N = 4 virtual elements per axis, d = lambda/2
    for (const auto & pk : peaks)
    {
      const Cell & cell = cells[pk.cell];
      emit_cell(s, cell, contribs, members_flat, pk.noise_est, beamwidth_rad, rng, out, diag);
    }
    lap(6);
    // false alarms (noise/clutter CFAR exceedances not otherwise modelled)
    const std::uint32_t nfa = rng.poisson(cal_.false_alarms_per_frame);
    for (std::uint32_t k = 0; k < nfa; ++k)
    {
      Detection d;
      double az = 0.0, el = 0.0;
      for (int tries = 0; tries < 64; ++tries)
      {
        az = -80.0 + 160.0 * rng.uniform(); el = -80.0 + 160.0 * rng.uniform();
        const double sa = std::sin(az * kPi / 180.0), se = std::sin(el * kPi / 180.0);
        if (sa * sa + se * se >= 0.98) {continue;}
        const double g = antenna_.one_way_rx_db(az, el);
        if (std::isfinite(g) && rng.uniform() < db2lin(g)) {break;}
      }
      const int rb = min_bin + static_cast<int>(rng.uniform() * std::max(1, max_bin - min_bin));
      const int db = static_cast<int>(rng.uniform() * nd) - nd / 2;
      const double r = rb * profile_.range_bin_m;
      const Vec3 u = direction(az * kPi / 180.0, el * kPi / 180.0);
      d.x = static_cast<float>(r * u.x); d.y = static_cast<float>(r * u.y); d.z = static_cast<float>(r * u.z);
      d.velocity = static_cast<float>(db * profile_.doppler_bin_mps);
      d.snr_db = static_cast<float>(10.0 * std::log10(threshold + rng.exponential()));
      d.noise_db = static_cast<float>(cal_.noise_reference_db + 10.0 * std::log10(noise_power));
      d.mechanism = Mechanism::FALSE_ALARM;
      d.range_bin = rb; d.doppler_bin = db;
      d.ideal_point_sensor = u * r;
      d.ideal_point_world = s.position_world + s.rotation_world_from_sensor.apply(d.ideal_point_sensor);
      out.push_back(d);
      ++diag.false_alarms;
    }
    lap(7);
    // processing output cap (strongest first), then deterministic order by range bin
    if (out.size() > proc_.max_points)
    {
      std::stable_sort(out.begin(), out.end(), [](const Detection & a, const Detection & b) {return a.snr_db > b.snr_db;});
      diag.capped = out.size() - proc_.max_points;
      out.resize(proc_.max_points);
    }
    std::stable_sort(out.begin(), out.end(), [](const Detection & a, const Detection & b) {
        return a.range_bin != b.range_bin ? a.range_bin < b.range_bin : a.doppler_bin < b.doppler_bin;});
    for (const auto & d : out) {diag.by_mechanism[static_cast<std::size_t>(d.mechanism)]++;}
    lap(8);
    if (diag_out) {*diag_out = diag;}
    return out;
  }

private:
  struct Cell
  {
    int range_bin{0}, doppler_bin{0};
    double re{0.0}, im{0.0}, power{0.0};
    std::uint32_t member_begin{0}, member_count{0};   // CSR span into the scan's member list
  };

  CellTrace trace_cell(const Cell & cell, const std::vector<Contribution> & contribs,
    const std::vector<std::uint32_t> & members_flat) const
  {
    const auto mb = members_flat.begin() + cell.member_begin;
    const std::vector<std::uint32_t> members(mb, mb + cell.member_count);
    CellTrace t;
    t.range_bin = cell.range_bin; t.doppler_bin = cell.doppler_bin; t.power = cell.power;
    t.coherent_power = cell.re * cell.re + cell.im * cell.im;
    t.members = static_cast<std::uint16_t>(std::min<std::size_t>(members.size(), 65535u));
    double best = -1.0;
    for (const auto m : members)
    {
      const auto & c = contribs[m];
      const auto k = static_cast<std::size_t>(c.mech);
      t.mech_power[k] += c.snr_lin;
      t.mech_count[k] = static_cast<std::uint16_t>(std::min(65535, t.mech_count[k] + 1));
      t.incoherent_power += c.snr_lin;
      if (c.snr_lin > best) {best = c.snr_lin; t.dominant_source = c.source_index;}
    }
    return t;
  }

  static double db2lin(double db) {return std::pow(10.0, db / 10.0);}
  static std::int64_t cell_key(int rb, int db) {return (static_cast<std::int64_t>(rb) << 20) ^ static_cast<std::int64_t>(db + 524288);}
  static int wrap_doppler(int b, int n)
  {
    int w = ((b + n / 2) % n + n) % n;
    return w - n / 2;
  }
  // Angles are direction-cosine angles, as measured by the AOP phase-comparison virtual array:
  // az = asin(u_y), el = asin(u_z) with u the unit line of sight in the sensor frame (x boresight).
  // Only the front hemisphere (u_x > 0) is valid; this avoids the atan2 azimuth singularity near +-90 deg elevation.
  static Vec3 direction(double az, double el)
  {
    const double wy = std::sin(az), wz = std::sin(el);
    const double wx2 = 1.0 - wy * wy - wz * wz;
    if (wx2 <= 0.0) {const double n = std::hypot(wy, wz); return {0.0, wy / n, wz / n};}
    return {std::sqrt(wx2), wy, wz};
  }
  static void angles(const Vec3 & p, double & az_deg, double & el_deg)
  {
    const double n = p.norm();
    if (n <= 0.0 || p.x <= 0.0)
    {
      az_deg = el_deg = std::numeric_limits<double>::quiet_NaN();
      return;
    }
    az_deg = std::asin(std::clamp(p.y / n, -1.0, 1.0)) * 180.0 / kPi;
    el_deg = std::asin(std::clamp(p.z / n, -1.0, 1.0)) * 180.0 / kPi;
  }

  int doppler_bin_of(double v) const
  {
    return wrap_doppler(static_cast<int>(std::lround(v / profile_.doppler_bin_mps)), profile_.doppler_fft);
  }

  // Common path: sensor-frame point -> contribution (returns false if invisible).
  bool make_contribution(
    const SensorState & s, const Vec3 & p_world, double sigma_m2, double phase0, Mechanism mech,
    int source, double cparam, Contribution & c, double extra_gain_db = 0.0) const
  {
    const Vec3 rel = p_world - s.position_world;
    const double r = rel.norm();
    if (r < 1e-3 || r > proc_.max_range_m + profile_.range_bin_m || sigma_m2 <= 0.0) {return false;}
    const Vec3 ps = s.rotation_world_from_sensor.apply_transpose(rel);
    double az, el;
    angles(ps, az, el);
    const double g = antenna_.two_way_db(az, el);
    if (!std::isfinite(g)) {return false;}
    const Vec3 u = rel * (1.0 / r);
    const double v_r = -s.velocity_world.dot(u);
    c.p_sensor = ps;
    c.gain_db = g;
    c.snr_lin = snr_constant_ * sigma_m2 / (r * r * r * r) * db2lin(g + extra_gain_db);
    c.phase = std::fmod(4.0 * kPi * r / profile_.lambda_m + phase0, 2.0 * kPi);
    c.range_bin = static_cast<int>(std::lround((r + cal_.range_bias_m) / profile_.range_bin_m));
    c.doppler_bin = doppler_bin_of(v_r);
    c.mech = mech; c.source_index = source; c.conductor_parameter = cparam;
    return true;
  }

  double scene_uniform(std::uint64_t a, std::uint64_t b) const
  {
    return (mix(mix(scene_seed_, a), b) >> 11) * (1.0 / 9007199254740992.0);
  }

  // conductor specular glints (+ ground-bounce ghosts) of conductor ci; consumes the scan RNG in conductor order
  void add_conductor_speculars(
    std::size_t ci, const SensorState & s, Rng & rng, std::vector<Contribution> & out, ScanDiagnostics & diag,
    std::vector<std::pair<double, Vec3>> & speculars) const
  {
    const double rmax = proc_.max_range_m + cal_.cull_margin_m;
    const auto & pts = conductors_[ci].samples_world;
    double prefix = 0.0;
    for (std::size_t i = 1; i < pts.size(); ++i)
    {
      const Vec3 a = pts[i - 1], b = pts[i], ab = b - a;
      const double len = ab.norm();
      if (len < 1e-9) {continue;}
      const double t = (s.position_world - a).dot(ab) / (len * len);
      const bool interior = t > 0.0 && t < 1.0;
      const bool vertex_min = !interior && t >= 1.0 && i + 1 < pts.size() &&
        (s.position_world - b).dot(pts[i + 1] - b) <= 0.0;
      if (interior || vertex_min)
      {
        const double tt = interior ? t : 1.0;
        const Vec3 p = a + ab * tt;
        if ((p - s.position_world).norm() <= rmax) {speculars.emplace_back(prefix + tt * len, p);}
      }
      prefix += len;
    }
    for (const auto & sp : speculars)
    {
      const double r = (sp.second - s.position_world).norm();
      const double sigma = kPi * cal_.conductor_radius_m * r * db2lin(cal_.conductor_specular_scale_db);
      Contribution c;
      const double fade = cal_.conductor_specular_fading_db * rng.normal();
      if (make_contribution(s, sp.second, sigma, 0.0, Mechanism::CONDUCTOR_SPECULAR, static_cast<int>(ci), sp.first, c, fade))
      {
        out.push_back(c);
        if (cal_.multipath_enabled) {add_ghost(s, sp.second, sigma, static_cast<int>(ci), sp.first, out, diag);}
      }
    }
  }

  // accelerator path: CPU speculars (same RNG order), accelerator diffuse/voxels/members, assembled in CPU order
  template<typename Lap>
  bool accelerated_candidates(
    const SensorState & s, std::uint64_t scan_sequence, Rng & rng, std::vector<Contribution> & out,
    ScanDiagnostics & diag, Lap & lap) const
  {
    const Rng rng_saved = rng;
    const ScanDiagnostics diag_saved = diag;
    std::vector<std::vector<Contribution>> spec(conductors_.size());
    std::vector<std::vector<double>> arcs(conductors_.size());
    for (std::size_t ci = 0; ci < conductors_.size(); ++ci)
    {
      std::vector<std::pair<double, Vec3>> speculars;
      add_conductor_speculars(ci, s, rng, spec[ci], diag, speculars);
      for (const auto & sp : speculars) {arcs[ci].push_back(sp.first);}
    }
    lap(0);
    std::vector<std::vector<Contribution>> diffuse;
    std::vector<Contribution> vox, mem;
    if (!accel_->evaluate(s, scan_sequence, arcs, diffuse, vox, mem) || diffuse.size() != conductors_.size())
    {
      rng = rng_saved; diag = diag_saved;       // fall back to the CPU reference path for this scan
      return false;
    }
    lap(1);
    out.clear();
    for (std::size_t ci = 0; ci < conductors_.size(); ++ci)
    {
      out.insert(out.end(), spec[ci].begin(), spec[ci].end());
      out.insert(out.end(), diffuse[ci].begin(), diffuse[ci].end());
    }
    out.insert(out.end(), vox.begin(), vox.end());
    out.insert(out.end(), mem.begin(), mem.end());
    lap(2);
    return true;
  }

  void add_conductors(const SensorState & s, Rng & rng, std::vector<Contribution> & out, ScanDiagnostics & diag) const
  {
    const double rmax = proc_.max_range_m + cal_.cull_margin_m;
    const double step = std::max(0.005, cal_.conductor_diffuse_step_fraction * profile_.range_bin_m);
    const double width = cal_.conductor_diffuse_aspect_width_deg * kPi / 180.0;
    const double diffuse_per_m = db2lin(cal_.conductor_diffuse_dbsm_per_m);
    for (std::size_t ci = 0; ci < conductors_.size(); ++ci)
    {
      const auto & pts = conductors_[ci].samples_world;
      // 1) specular glints at stationary points of range along the polyline
      std::vector<std::pair<double, Vec3>> speculars;   // (arc parameter, point)
      add_conductor_speculars(ci, s, rng, out, diag, speculars);
      double prefix = 0.0;
      // 2) diffuse elements (strand/roughness), aspect-dependent; Fresnel zone around glints excluded
      if (diffuse_static_valid_)
      {
        // r22 Backlog 03: static element positions/arc/phase precomputed at configure (identical values)
        const auto & segs = diffuse_segments_[ci];
        for (const auto & sg : segs)
        {
          const double t = std::clamp((s.position_world - sg.a).dot(sg.ab) / (sg.len * sg.len), 0.0, 1.0);
          if ((sg.a + sg.ab * t - s.position_world).norm() > rmax) {continue;}
          for (std::uint32_t e = sg.first; e < sg.first + sg.n; ++e)
          {
            const auto & el = diffuse_elements_[e];
            const Vec3 rel = el.p - s.position_world;
            const double r = rel.norm();
            if (r > rmax || r < 1e-3) {continue;}
            bool in_fresnel = false;
            const double fresnel_half = 0.5 * std::sqrt(profile_.lambda_m * r / 2.0);
            for (const auto & sp : speculars) {if (std::abs(el.arc - sp.first) < fresnel_half) {in_fresnel = true; break;}}
            if (in_fresnel) {continue;}
            const double sin_psi = std::abs(sg.tangent.dot(rel) / r);
            const double psi = std::asin(std::min(1.0, sin_psi));
            const double aspect = (1.0 - sin_psi * sin_psi) * std::exp(-(psi / width) * (psi / width));
            const double sigma = diffuse_per_m * sg.ds * aspect;
            if (sigma < 1e-12) {continue;}
            Contribution c;
            if (make_contribution(s, el.p, sigma, el.phase0, Mechanism::CONDUCTOR_DIFFUSE, static_cast<int>(ci), el.arc, c))
            {
              out.push_back(c);
            }
          }
        }
        continue;
      }
      prefix = 0.0;
      std::uint64_t element = 0;
      for (std::size_t i = 1; i < pts.size(); ++i)
      {
        const Vec3 a = pts[i - 1], b = pts[i], ab = b - a;
        const double len = ab.norm();
        if (len < 1e-9) {continue;}
        const Vec3 tangent = ab * (1.0 / len);
        // quick segment cull
        const double t = std::clamp((s.position_world - a).dot(ab) / (len * len), 0.0, 1.0);
        if ((a + ab * t - s.position_world).norm() > rmax) {prefix += len; element += static_cast<std::uint64_t>(len / step) + 1; continue;}
        const int n = std::max(1, static_cast<int>(std::ceil(len / step)));
        const double ds = len / n;
        for (int k = 0; k < n; ++k, ++element)
        {
          const double arc = prefix + (k + 0.5) * ds;
          const Vec3 p = a + tangent * ((k + 0.5) * ds);
          const Vec3 rel = p - s.position_world;
          const double r = rel.norm();
          if (r > rmax || r < 1e-3) {continue;}
          bool in_fresnel = false;
          const double fresnel_half = 0.5 * std::sqrt(profile_.lambda_m * r / 2.0);
          for (const auto & sp : speculars) {if (std::abs(arc - sp.first) < fresnel_half) {in_fresnel = true; break;}}
          if (in_fresnel) {continue;}
          const double sin_psi = std::abs(tangent.dot(rel) / r);
          const double psi = std::asin(std::min(1.0, sin_psi));
          const double aspect = (1.0 - sin_psi * sin_psi) * std::exp(-(psi / width) * (psi / width));
          const double sigma = diffuse_per_m * ds * aspect;
          if (sigma < 1e-12) {continue;}
          Contribution c;
          const double phase0 = 2.0 * kPi * scene_uniform(0x1000u + ci, element);
          if (make_contribution(s, p, sigma, phase0, Mechanism::CONDUCTOR_DIFFUSE, static_cast<int>(ci), arc, c))
          {
            out.push_back(c);
          }
        }
        prefix += len;
      }
    }
  }

  void add_ghost(
    const SensorState & s, const Vec3 & p, double sigma, int ci, double arc,
    std::vector<Contribution> & out, ScanDiagnostics & diag) const
  {
    const Vec3 image{p.x, p.y, 2.0 * cal_.terrain_z_m - p.z};
    const double r1 = (p - s.position_world).norm(), r2 = (image - s.position_world).norm();
    const Vec3 ps1 = s.rotation_world_from_sensor.apply_transpose(p - s.position_world);
    const Vec3 ps2 = s.rotation_world_from_sensor.apply_transpose(image - s.position_world);
    double az1, el1, az2, el2;
    angles(ps1, az1, el1); angles(ps2, az2, el2);
    const double g1 = antenna_.two_way_db(az1, el1), g2 = antenna_.two_way_db(az2, el2);
    if (!std::isfinite(g1) || !std::isfinite(g2)) {return;}
    const double r_app = 0.5 * (r1 + r2);
    if (r_app > proc_.max_range_m) {return;}
    const Vec3 u2 = (image - s.position_world) * (1.0 / r2);
    const Vec3 u1 = (p - s.position_world) * (1.0 / r1);
    const double v = -0.5 * (s.velocity_world.dot(u1) + s.velocity_world.dot(u2));
    Contribution c;
    c.p_sensor = ps2 * (r_app / r2);
    c.gain_db = 0.5 * (g1 + g2);
    c.snr_lin = snr_constant_ * sigma / (r1 * r1 * r2 * r2) * db2lin(c.gain_db + cal_.multipath_ground_reflection_db);
    c.phase = std::fmod(2.0 * kPi * (r1 + r2) / profile_.lambda_m, 2.0 * kPi);
    c.range_bin = static_cast<int>(std::lround(r_app / profile_.range_bin_m));
    c.doppler_bin = doppler_bin_of(v);
    c.mech = Mechanism::MULTIPATH_GHOST; c.source_index = ci; c.conductor_parameter = arc;
    out.push_back(c);
    ++diag.ghosts;
  }

  // r22: specular glint of each straight lattice member at the foot point of the perpendicular from the
  // sensor (monostatic specular condition LOS _|_ member axis), exactly as for conductors; thin-cylinder
  // broadside RCS min(pi a R, 2 pi a L^2 / lambda) (Fresnel-region / finite-length limits).
  void add_members(const SensorState & s, std::uint64_t scan_sequence, std::vector<Contribution> & out) const
  {
    if (members_.empty()) {return;}
    const double rmax = proc_.max_range_m + cal_.cull_margin_m;
    const double scale = db2lin(cal_.pylon_member_scale_db);
    for (std::size_t i = 0; i < members_.size(); ++i)
    {
      const auto & m = members_[i];
      const Vec3 ab = m.b - m.a;
      const double len2 = ab.dot(ab);
      if (len2 < 1e-12) {continue;}
      const double t = (s.position_world - m.a).dot(ab) / len2;
      if (t <= 0.0 || t >= 1.0) {continue;}
      const Vec3 p = m.a + ab * t;
      const double r = (p - s.position_world).norm();
      if (r > rmax || r < 1e-3) {continue;}
      const double sigma = std::min(kPi * m.radius_m * r, 2.0 * kPi * m.radius_m * len2 / profile_.lambda_m) *
        member_factor_[i] * scale;
      // per-scan fading from a counter-based stream (does not perturb the shared scan RNG)
      Rng fr(mix(mix(stream_seed_, scan_sequence), 0x7000u + i));
      const double fade = cal_.pylon_member_fading_db * fr.normal();
      Contribution c;
      if (make_contribution(s, p, sigma, 0.0, Mechanism::PYLON, m.pylon_index, std::numeric_limits<double>::quiet_NaN(), c, fade))
      {
        out.push_back(c);
      }
    }
  }

  // r22 Backlog 03: static per-scene data precomputed once (values identical to the per-scan expressions).
  void precompute_static()
  {
    const double rho_pylon = db2lin(cal_.pylon_reflectivity_db);
    const double rho_struct = db2lin(cal_.structure_reflectivity_db);
    voxel_sigma_.assign(voxels_.size(), 0.0);
    voxel_phase0_.resize(voxels_.size());
    for (std::size_t i = 0; i < voxels_.size(); ++i)
    {
      const auto & v = voxels_[i];
      if (v.cls == 0) {voxel_sigma_[i] = rho_pylon * v.area_m2 * voxel_factor_[i];}
      else if (v.cls == 1) {voxel_sigma_[i] = rho_struct * v.area_m2 * voxel_factor_[i];}
      voxel_phase0_[i] = 2.0 * kPi * scene_uniform(0x4000u, i);
    }
    // conductor diffuse elements; exact only if no segment length is an integral number of steps
    // (the dynamic path numbers elements of culled segments as floor(len/step)+1, of others as ceil(len/step))
    const double step = std::max(0.005, cal_.conductor_diffuse_step_fraction * profile_.range_bin_m);
    diffuse_segments_.assign(conductors_.size(), {});
    diffuse_elements_.clear();
    diffuse_static_valid_ = true;
    for (std::size_t ci = 0; ci < conductors_.size(); ++ci)
    {
      const auto & pts = conductors_[ci].samples_world;
      double prefix = 0.0;
      std::uint64_t element = 0;
      for (std::size_t i = 1; i < pts.size(); ++i)
      {
        const Vec3 a = pts[i - 1], b = pts[i], ab = b - a;
        const double len = ab.norm();
        if (len < 1e-9) {continue;}
        const int n = std::max(1, static_cast<int>(std::ceil(len / step)));
        if (static_cast<std::uint64_t>(len / step) + 1 != static_cast<std::uint64_t>(n)) {diffuse_static_valid_ = false;}
        const Vec3 tangent = ab * (1.0 / len);
        const double ds = len / n;
        DiffuseSegment sg{a, ab, tangent, len, ds, static_cast<std::uint32_t>(diffuse_elements_.size()), static_cast<std::uint32_t>(n)};
        for (int k = 0; k < n; ++k, ++element)
        {
          DiffuseElement el;
          el.arc = prefix + (k + 0.5) * ds;
          el.p = a + tangent * ((k + 0.5) * ds);
          el.phase0 = 2.0 * kPi * scene_uniform(0x1000u + ci, element);
          diffuse_elements_.push_back(el);
        }
        diffuse_segments_[ci].push_back(sg);
        prefix += len;
      }
    }
    // dense range-Doppler grid rows: every contribution range bin plus CFAR/peak neighbourhoods
    const double rmax_contrib = proc_.max_range_m + profile_.range_bin_m + std::abs(cal_.range_bias_m) + cal_.cull_margin_m;
    grid_rows_ = static_cast<int>(std::ceil(rmax_contrib / profile_.range_bin_m)) + 2 * kGridPad + 2;
  }

  void build_buckets()
  {
    buckets_.clear();
    for (std::size_t i = 0; i < voxels_.size(); ++i)
    {
      buckets_[bucket_key(voxels_[i].p.x, voxels_[i].p.y)].push_back(static_cast<std::uint32_t>(i));
    }
    voxel_factor_.resize(voxels_.size());
    for (std::size_t i = 0; i < voxels_.size(); ++i)
    {
      const auto & v = voxels_[i];
      const double sd = v.cls == 0 ? cal_.pylon_lognormal_db : (v.cls == 1 ? cal_.structure_lognormal_db : 0.0);
      // static per-scatterer lognormal reflectivity (scene property, shared by all radars)
      const double u1 = std::max(1e-12, scene_uniform(0x2000u, i)), u2 = scene_uniform(0x3000u, i);
      const double n = std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * kPi * u2);
      voxel_factor_[i] = db2lin(sd * n);
    }
  }
  static constexpr double kBucket = 4.0;
  static std::int64_t bucket_key(double x, double y)
  {
    const auto ix = static_cast<std::int64_t>(std::floor(x / kBucket));
    const auto iy = static_cast<std::int64_t>(std::floor(y / kBucket));
    return (ix << 32) ^ (iy & 0xffffffff);
  }

  void add_voxels(const SensorState & s, std::vector<Contribution> & out) const
  {
    const double rmax = proc_.max_range_m + cal_.cull_margin_m;
    const auto ix0 = static_cast<std::int64_t>(std::floor((s.position_world.x - rmax) / kBucket));
    const auto ix1 = static_cast<std::int64_t>(std::floor((s.position_world.x + rmax) / kBucket));
    const auto iy0 = static_cast<std::int64_t>(std::floor((s.position_world.y - rmax) / kBucket));
    const auto iy1 = static_cast<std::int64_t>(std::floor((s.position_world.y + rmax) / kBucket));
    const double gamma = db2lin(cal_.terrain_gamma_db);
    for (auto ix = ix0; ix <= ix1; ++ix)
    {
      for (auto iy = iy0; iy <= iy1; ++iy)
      {
        auto it = buckets_.find((ix << 32) ^ (iy & 0xffffffff));
        if (it == buckets_.end()) {continue;}
        for (const auto i : it->second)
        {
          const auto & v = voxels_[i];
          const Vec3 rel = v.p - s.position_world;
          const double r = rel.norm();
          if (r > rmax || r < 1e-3) {continue;}
          double sigma;
          Mechanism mech;
          if (v.cls == 0) {sigma = voxel_sigma_[i]; mech = Mechanism::PYLON;}
          else if (v.cls == 1) {sigma = voxel_sigma_[i]; mech = Mechanism::STRUCTURE;}
          else
          {
            const double sin_graze = std::max(0.0, -rel.z / r);   // terrain seen from above
            if (sin_graze <= 0.0) {continue;}
            sigma = gamma * sin_graze * v.area_m2; mech = Mechanism::TERRAIN;
          }
          Contribution c;
          const double phase0 = voxel_phase0_[i];
          if (make_contribution(s, v.p, sigma, phase0, mech, v.cls == 0 ? v.pylon_index : -1,
              std::numeric_limits<double>::quiet_NaN(), c))
          {
            out.push_back(c);
          }
        }
      }
    }
  }

  void emit_cell(
    const SensorState & s, const Cell & cell, const std::vector<Contribution> & contribs,
    const std::vector<std::uint32_t> & members_flat, double noise_est,
    double beamwidth, Rng & rng, std::vector<Detection> & out, ScanDiagnostics & /*diag*/) const
  {
    // angular clustering of the cell's scatterers (strongest first)
    std::vector<std::size_t> order(members_flat.begin() + cell.member_begin, members_flat.begin() + cell.member_begin + cell.member_count);
    std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {return contribs[a].snr_lin > contribs[b].snr_lin;});
    struct Cluster {Vec3 dir_sum; double power{0.0}; std::size_t lead; std::size_t count{0}; std::array<double, 8> mech_power{};};
    std::vector<Cluster> clusters;
    for (const auto m : order)
    {
      const auto & c = contribs[m];
      const Vec3 u = c.p_sensor * (1.0 / std::max(1e-9, c.p_sensor.norm()));
      bool placed = false;
      for (auto & cl : clusters)
      {
        const Vec3 lead_u = contribs[cl.lead].p_sensor * (1.0 / std::max(1e-9, contribs[cl.lead].p_sensor.norm()));
        if (std::acos(std::clamp(u.dot(lead_u), -1.0, 1.0)) < beamwidth)
        {
          cl.dir_sum = cl.dir_sum + u * c.snr_lin; cl.power += c.snr_lin; cl.count++;
          cl.mech_power[static_cast<std::size_t>(c.mech)] += c.snr_lin; placed = true; break;
        }
      }
      if (!placed)
      {
        Cluster cl; cl.dir_sum = u * c.snr_lin; cl.power = c.snr_lin; cl.lead = m; cl.count = 1;
        cl.mech_power[static_cast<std::size_t>(c.mech)] = c.snr_lin;
        clusters.push_back(cl);
      }
    }
    if (clusters.empty()) {return;}
    std::stable_sort(clusters.begin(), clusters.end(), [](const Cluster & a, const Cluster & b) {return a.power > b.power;});
    double total = 0.0;
    for (const auto & cl : clusters) {total += cl.power;}
    const std::size_t emit = (proc_.multi_object_beamforming > 0.0 && clusters.size() > 1 &&
      clusters[1].power >= proc_.multi_object_beamforming * clusters[0].power) ? 2u : 1u;
    const double snr_meas = cell.power / std::max(1e-12, noise_est);
    for (std::size_t k = 0; k < emit; ++k)
    {
      const Cluster & cl = clusters[k];
      const double share = cl.power / std::max(1e-12, total);
      const Vec3 dir = cl.dir_sum * (1.0 / std::max(1e-12, cl.dir_sum.norm()));
      const double cluster_snr = std::max(1e-3, snr_meas * share);
      const double crb = beamwidth / (1.6 * std::sqrt(2.0 * cluster_snr));
      const double floor = cal_.angle_floor_deg * kPi / 180.0;
      const double sigma_w = std::sqrt(crb * crb + floor * floor);   // direction-cosine noise (rad at boresight)
      double wy = dir.y + rng.normal() * sigma_w;
      double wz = dir.z + rng.normal() * sigma_w;
      const double wn = wy * wy + wz * wz;
      if (wn > 0.999) {const double s = std::sqrt(0.999 / wn); wy *= s; wz *= s;}
      const double r = cell.range_bin * profile_.range_bin_m;
      const Vec3 u{std::sqrt(std::max(0.0, 1.0 - wy * wy - wz * wz)), wy, wz};
      Detection d;
      d.x = static_cast<float>(r * u.x); d.y = static_cast<float>(r * u.y); d.z = static_cast<float>(r * u.z);
      d.velocity = static_cast<float>(cell.doppler_bin * profile_.doppler_bin_mps);
      // TI reports the cell SNR; a second beamformed object in the same cell carries its power share.
      d.snr_db = static_cast<float>(10.0 * std::log10(std::max(1e-6, k == 0 ? snr_meas : snr_meas * share)));
      d.noise_db = static_cast<float>(cal_.noise_reference_db + 10.0 * std::log10(noise_est));
      std::size_t best = 0;
      for (std::size_t m = 1; m < cl.mech_power.size(); ++m) {if (cl.mech_power[m] > cl.mech_power[best]) {best = m;}}
      d.mechanism = static_cast<Mechanism>(best);
      const auto & lead = contribs[cl.lead];
      d.source_index = lead.source_index;
      d.dominant_power_fraction = cl.mech_power[best] / std::max(1e-12, total);
      d.scatterers_in_cell = static_cast<std::uint16_t>(std::min<std::size_t>(cell.member_count, 65535u));
      d.ideal_point_sensor = lead.p_sensor;
      d.ideal_point_world = s.position_world + s.rotation_world_from_sensor.apply(lead.p_sensor);
      d.conductor_parameter_m = lead.conductor_parameter;
      d.two_way_gain_db = lead.gain_db;
      d.mean_snr_db = 10.0 * std::log10(std::max(1e-12, cl.power));
      d.range_bin = cell.range_bin; d.doppler_bin = cell.doppler_bin;
      out.push_back(d);
    }
  }

  FmcwProfile profile_;
  AntennaPattern antenna_;
  Calibration cal_;
  Processing proc_;
  std::vector<Conductor> conductors_;
  std::vector<VoxelScatterer> voxels_;
  std::vector<double> voxel_factor_;
  std::shared_ptr<CandidateAccelerator> accel_;
  std::vector<MemberSegment> members_;
  struct DiffuseSegment {Vec3 a, ab, tangent; double len, ds; std::uint32_t first, n;};
  struct DiffuseElement {Vec3 p; double arc{0.0}, phase0{0.0};};
  std::vector<std::vector<DiffuseSegment>> diffuse_segments_;
  std::vector<DiffuseElement> diffuse_elements_;
  bool diffuse_static_valid_{false};
  std::vector<double> voxel_sigma_, voxel_phase0_;
  static constexpr int kGridPad = 32;
  int grid_rows_{0};
  std::vector<double> member_factor_;
  std::unordered_map<std::int64_t, std::vector<std::uint32_t>> buckets_;
  std::uint64_t stream_seed_{0}, scene_seed_{0};
  double snr_constant_{0.0};
  bool configured_{false};
};

}  // namespace aop_radar
