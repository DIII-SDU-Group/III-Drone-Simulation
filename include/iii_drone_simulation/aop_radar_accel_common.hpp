// r22 / WO-2026-09-29-002 Backlog 03: candidate-evaluation math shared by the CPU-emulated and CUDA accelerators.
//
// Every function mirrors the CPU reference (aop_radar_model.hpp) operation by operation so that the host
// instantiation is bitwise identical to it; the CUDA instantiation differs only by device libm rounding
// (asin/pow/exp/log/sin/cos, <= a few ulp).  Compile device code with -fmad=false (no FMA contraction).
#pragma once

#include <cmath>
#include <math.h>
#include <cstdint>

#ifdef __CUDACC__
#define AOP_HD __host__ __device__
#else
#define AOP_HD
#endif

namespace aop_accel
{

constexpr double kPi = 3.14159265358979323846;
constexpr int kMaxAngles = 32;

struct V3 {double x, y, z;};
AOP_HD inline V3 sub(V3 a, V3 b) {return {a.x - b.x, a.y - b.y, a.z - b.z};}
AOP_HD inline V3 add(V3 a, V3 b) {return {a.x + b.x, a.y + b.y, a.z + b.z};}
AOP_HD inline V3 mul(V3 a, double s) {return {a.x * s, a.y * s, a.z * s};}
AOP_HD inline double dot(V3 a, V3 b) {return a.x * b.x + a.y * b.y + a.z * b.z;}
AOP_HD inline double norm(V3 a) {return sqrt(dot(a, a));}

// layout identical to aop_radar::Contribution (checked by static_assert in the host code)
struct Contrib
{
  V3 p_sensor;
  double snr_lin, phase, gain_db;
  int range_bin, doppler_bin;
  std::uint8_t mech;
  int source_index;
  double conductor_parameter;
};

struct Sensor {V3 pos; double R[9]; V3 vel;};

struct Consts
{
  double lambda, range_bin_m, doppler_bin_mps, max_range, rmax, snr_constant, range_bias;
  double diffuse_width, diffuse_per_m, terrain_gamma, member_scale, member_fading_db;
  int doppler_fft, n_angles;
  std::uint64_t stream_seed;
  double angles[kMaxAngles], tx_az[kMaxAngles], rx_az[kMaxAngles], tx_el[kMaxAngles], rx_el[kMaxAngles];
};

struct Seg {V3 a, ab, tangent; double len, ds;};
struct Elem {V3 p; double arc, phase0; std::uint32_t segment, conductor;};
struct Vox {V3 p; double area, sigma_base, phase0; int cls, pylon_index;};
struct Mem {V3 a, b; double radius, factor; int pylon_index;};

AOP_HD inline double db2lin(double db) {return pow(10.0, db / 10.0);}

AOP_HD inline double interp(const double * xs, const double * ys, int n, double x)
{
  if (!(x >= xs[0] && x <= xs[n - 1])) {return NAN;}
  int lo = 0, hi = n;                       // std::upper_bound
  while (lo < hi) {const int mid = (lo + hi) / 2; if (xs[mid] <= x) {lo = mid + 1;} else {hi = mid;}}
  if (lo == n) {return ys[n - 1];}
  const int i = lo;
  const double t = (x - xs[i - 1]) / (xs[i] - xs[i - 1]);
  return ys[i - 1] + t * (ys[i] - ys[i - 1]);
}

AOP_HD inline double two_way_db(const Consts & k, double az, double el)
{
  return interp(k.angles, k.tx_az, k.n_angles, az) + interp(k.angles, k.rx_az, k.n_angles, az) +
         interp(k.angles, k.tx_el, k.n_angles, el) + interp(k.angles, k.rx_el, k.n_angles, el);
}

AOP_HD inline int wrap_doppler(int b, int n)
{
  int w = ((b + n / 2) % n + n) % n;
  return w - n / 2;
}

AOP_HD inline double clampd(double v, double lo, double hi) {return v < lo ? lo : (v > hi ? hi : v);}

// mirrors AopRadarModel::make_contribution
AOP_HD inline bool make_contribution(
  const Consts & k, const Sensor & s, V3 p_world, double sigma_m2, double phase0, std::uint8_t mech, int source,
  double cparam, Contrib & c, double extra_gain_db)
{
  const V3 rel = sub(p_world, s.pos);
  const double r = norm(rel);
  if (r < 1e-3 || r > k.max_range + k.range_bin_m || sigma_m2 <= 0.0) {return false;}
  const V3 ps = {s.R[0] * rel.x + s.R[3] * rel.y + s.R[6] * rel.z, s.R[1] * rel.x + s.R[4] * rel.y + s.R[7] * rel.z,
    s.R[2] * rel.x + s.R[5] * rel.y + s.R[8] * rel.z};
  const double n = norm(ps);
  if (n <= 0.0 || ps.x <= 0.0) {return false;}
  const double az = asin(clampd(ps.y / n, -1.0, 1.0)) * 180.0 / kPi;
  const double el = asin(clampd(ps.z / n, -1.0, 1.0)) * 180.0 / kPi;
  const double g = two_way_db(k, az, el);
  if (!isfinite(g)) {return false;}
  const V3 u = mul(rel, 1.0 / r);
  const double v_r = -dot(s.vel, u);
  c.p_sensor = ps;
  c.gain_db = g;
  c.snr_lin = k.snr_constant * sigma_m2 / (r * r * r * r) * db2lin(g + extra_gain_db);
  c.phase = fmod(4.0 * kPi * r / k.lambda + phase0, 2.0 * kPi);
  c.range_bin = static_cast<int>(lround((r + k.range_bias) / k.range_bin_m));
  c.doppler_bin = wrap_doppler(static_cast<int>(lround(v_r / k.doppler_bin_mps)), k.doppler_fft);
  c.mech = mech; c.source_index = source; c.conductor_parameter = cparam;
  return true;
}

// conductor diffuse element (CPU: add_conductors precomputed path); arcs = this conductor's specular arcs
AOP_HD inline bool eval_diffuse(
  const Consts & k, const Sensor & s, const Seg & sg, const Elem & el, const double * arcs, int n_arcs, Contrib & c)
{
  const double t = clampd(dot(sub(s.pos, sg.a), sg.ab) / (sg.len * sg.len), 0.0, 1.0);
  if (norm(sub(add(sg.a, mul(sg.ab, t)), s.pos)) > k.rmax) {return false;}
  const V3 rel = sub(el.p, s.pos);
  const double r = norm(rel);
  if (r > k.rmax || r < 1e-3) {return false;}
  const double fresnel_half = 0.5 * sqrt(k.lambda * r / 2.0);
  for (int i = 0; i < n_arcs; ++i) {if (fabs(el.arc - arcs[i]) < fresnel_half) {return false;}}
  const double sin_psi = fabs(dot(sg.tangent, rel) / r);
  const double psi = asin(sin_psi < 1.0 ? sin_psi : 1.0);
  const double aspect = (1.0 - sin_psi * sin_psi) * exp(-(psi / k.diffuse_width) * (psi / k.diffuse_width));
  const double sigma = k.diffuse_per_m * sg.ds * aspect;
  if (sigma < 1e-12) {return false;}
  return make_contribution(k, s, el.p, sigma, el.phase0, 2 /*CONDUCTOR_DIFFUSE*/, static_cast<int>(el.conductor), el.arc, c, 0.0);
}

// voxel (CPU: add_voxels); cls 0 pylon, 1 structure, 2 terrain
AOP_HD inline bool eval_voxel(const Consts & k, const Sensor & s, const Vox & v, Contrib & c)
{
  const V3 rel = sub(v.p, s.pos);
  const double r = norm(rel);
  if (r > k.rmax || r < 1e-3) {return false;}
  double sigma;
  std::uint8_t mech;
  if (v.cls == 0) {sigma = v.sigma_base; mech = 3;}
  else if (v.cls == 1) {sigma = v.sigma_base; mech = 5;}
  else
  {
    const double q = -rel.z / r;
    const double sin_graze = q > 0.0 ? q : 0.0;
    if (sin_graze <= 0.0) {return false;}
    sigma = k.terrain_gamma * sin_graze * v.area; mech = 4;
  }
  return make_contribution(k, s, v.p, sigma, v.phase0, mech, v.cls == 0 ? v.pylon_index : -1, NAN, c, 0.0);
}

// counter-based RNG identical to aop_radar::Rng / mix
AOP_HD inline std::uint64_t splitmix64(std::uint64_t x)
{
  x += 0x9E3779B97F4A7C15ULL;
  x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ULL;
  x = (x ^ (x >> 27)) * 0x94D049BB133111EBULL;
  return x ^ (x >> 31);
}
AOP_HD inline std::uint64_t mix(std::uint64_t a, std::uint64_t b) {return splitmix64(a ^ splitmix64(b + 0x632BE59BD9B4E019ULL));}
AOP_HD inline std::uint64_t rotl(std::uint64_t x, int r) {return (x << r) | (x >> (64 - r));}
struct Rng
{
  std::uint64_t s[4];
  AOP_HD explicit Rng(std::uint64_t seed) {for (int i = 0; i < 4; ++i) {seed = splitmix64(seed); s[i] = seed;}}
  AOP_HD std::uint64_t next()
  {
    const std::uint64_t result = rotl(s[1] * 5, 7) * 9;
    const std::uint64_t t = s[1] << 17;
    s[2] ^= s[0]; s[3] ^= s[1]; s[1] ^= s[2]; s[0] ^= s[3]; s[2] ^= t; s[3] = rotl(s[3], 45);
    return result;
  }
  AOP_HD double uniform() {return (next() >> 11) * (1.0 / 9007199254740992.0);}
  AOP_HD double normal()   // first draw of a fresh stream (cos branch), as the CPU Rng
  {
    double u1 = uniform();
    while (u1 <= 1e-300) {u1 = uniform();}
    const double u2 = uniform();
    const double r = sqrt(-2.0 * log(u1));
    return r * cos(2.0 * kPi * u2);
  }
};

// pylon member (CPU: add_members)
AOP_HD inline bool eval_member(
  const Consts & k, const Sensor & s, const Mem & m, std::uint32_t index, std::uint64_t scan_sequence, Contrib & c)
{
  const V3 ab = sub(m.b, m.a);
  const double len2 = dot(ab, ab);
  if (len2 < 1e-12) {return false;}
  const double t = dot(sub(s.pos, m.a), ab) / len2;
  if (t <= 0.0 || t >= 1.0) {return false;}
  const V3 p = add(m.a, mul(ab, t));
  const double r = norm(sub(p, s.pos));
  if (r > k.rmax || r < 1e-3) {return false;}
  const double a1 = kPi * m.radius * r, a2 = 2.0 * kPi * m.radius * len2 / k.lambda;
  const double sigma = (a1 < a2 ? a1 : a2) * m.factor * k.member_scale;
  Rng fr(mix(mix(k.stream_seed, scan_sequence), 0x7000u + index));
  const double fade = k.member_fading_db * fr.normal();
  return make_contribution(k, s, p, sigma, 0.0, 3 /*PYLON*/, m.pylon_index, NAN, c, fade);
}

}  // namespace aop_accel
