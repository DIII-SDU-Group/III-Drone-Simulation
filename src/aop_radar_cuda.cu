// r22 / WO-2026-09-29-002 Backlog 03: CUDA backend for the simulator-v2 candidate stage.
//
// Static scene data (conductor diffuse elements, voxels in CPU bucket order, pylon members, antenna tables)
// stay resident on the device.  Per scan only the sensor state and the conductor specular arcs are uploaded;
// one thread evaluates one candidate with the shared math of aop_radar_accel_common.hpp, an order-preserving
// stream compaction (cub::DeviceSelect::Flagged) keeps the CPU candidate order, a segmented flag sum gives the
// per-conductor / voxel / member counts, and only the compact valid contributions are copied back.
// No atomics or floating-point reductions: the backend is deterministic run to run.  Build with -fmad=false.
#include <cub/device/device_segmented_reduce.cuh>
#include <cub/device/device_select.cuh>
#include <cuda_runtime.h>

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "iii_drone_simulation/aop_radar_accel.hpp"

namespace aop_radar
{
namespace
{

#define AOP_CUDA_CHECK(x) do {const cudaError_t e_ = (x); if (e_ != cudaSuccess) {throw std::runtime_error(std::string(#x) + ": " + cudaGetErrorString(e_));}} while (0)

__global__ void evaluate_kernel(
  const aop_accel::Consts * k, const aop_accel::Sensor * s, std::uint64_t scan_sequence,
  const aop_accel::Seg * segs, const aop_accel::Elem * elems, std::uint32_t n_elems,
  const aop_accel::Vox * vox, std::uint32_t n_vox, const aop_accel::Mem * mem, std::uint32_t n_mem,
  const double * arcs, const int * arc_begin, aop_accel::Contrib * out, int * flags)
{
  const std::uint32_t i = blockIdx.x * blockDim.x + threadIdx.x;
  const std::uint32_t n = n_elems + n_vox + n_mem;
  if (i >= n) {return;}
  aop_accel::Contrib c{};
  bool ok;
  if (i < n_elems)
  {
    const auto & el = elems[i];
    const int b = arc_begin[el.conductor], e = arc_begin[el.conductor + 1];
    ok = aop_accel::eval_diffuse(*k, *s, segs[el.segment], el, arcs + b, e - b, c);
  }
  else if (i < n_elems + n_vox)
  {
    ok = aop_accel::eval_voxel(*k, *s, vox[i - n_elems], c);
  }
  else
  {
    const std::uint32_t m = i - n_elems - n_vox;
    ok = aop_accel::eval_member(*k, *s, mem[m], m, scan_sequence, c);
  }
  out[i] = c;
  flags[i] = ok ? 1 : 0;
}

template<typename T>
T * upload(const std::vector<T> & v)
{
  T * d = nullptr;
  const std::size_t bytes = std::max<std::size_t>(1, v.size()) * sizeof(T);
  AOP_CUDA_CHECK(cudaMalloc(&d, bytes));
  if (!v.empty()) {AOP_CUDA_CHECK(cudaMemcpy(d, v.data(), v.size() * sizeof(T), cudaMemcpyHostToDevice));}
  return d;
}

class CudaAccelerator : public CandidateAccelerator
{
public:
  explicit CudaAccelerator(const SceneExport & e)
  {
    a_ = make_accel_scene(e);
    n_elems_ = static_cast<std::uint32_t>(a_.elems.size());
    n_vox_ = static_cast<std::uint32_t>(a_.vox.size());
    n_mem_ = static_cast<std::uint32_t>(a_.mem.size());
    n_ = n_elems_ + n_vox_ + n_mem_;
    n_cond_ = a_.conductor_begin.size() - 1;
    AOP_CUDA_CHECK(cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking));
    d_k_ = upload(std::vector<aop_accel::Consts>{a_.k});
    d_segs_ = upload(a_.segs); d_elems_ = upload(a_.elems); d_vox_ = upload(a_.vox); d_mem_ = upload(a_.mem);
    AOP_CUDA_CHECK(cudaMalloc(&d_sensor_, sizeof(aop_accel::Sensor)));
    AOP_CUDA_CHECK(cudaMalloc(&d_out_, std::max<std::size_t>(1, n_) * sizeof(aop_accel::Contrib)));
    AOP_CUDA_CHECK(cudaMalloc(&d_compact_, std::max<std::size_t>(1, n_) * sizeof(aop_accel::Contrib)));
    AOP_CUDA_CHECK(cudaMalloc(&d_flags_, std::max<std::size_t>(1, n_) * sizeof(int)));
    // segment offsets over the candidate index space: conductors..., voxels, members
    std::vector<int> offs;
    for (auto b : a_.conductor_begin) {offs.push_back(static_cast<int>(b));}
    offs.push_back(static_cast<int>(n_elems_ + n_vox_));
    offs.push_back(static_cast<int>(n_));
    n_segments_ = static_cast<int>(offs.size()) - 1;
    d_offsets_ = upload(offs);
    AOP_CUDA_CHECK(cudaMalloc(&d_counts_, (n_segments_ + 1) * sizeof(int)));
    AOP_CUDA_CHECK(cudaMalloc(&d_arc_begin_, (n_cond_ + 1) * sizeof(int)));
    arcs_capacity_ = 64;
    AOP_CUDA_CHECK(cudaMalloc(&d_arcs_, arcs_capacity_ * sizeof(double)));
    std::size_t t1 = 0, t2 = 0;
    AOP_CUDA_CHECK(cub::DeviceSelect::Flagged(nullptr, t1, d_out_, d_flags_, d_compact_, d_counts_ + n_segments_, static_cast<int>(n_), stream_));
    AOP_CUDA_CHECK(cub::DeviceSegmentedReduce::Sum(nullptr, t2, d_flags_, d_counts_, n_segments_, d_offsets_, d_offsets_ + 1, stream_));
    temp_bytes_ = std::max(t1, t2);
    AOP_CUDA_CHECK(cudaMalloc(&d_temp_, std::max<std::size_t>(1, temp_bytes_)));
    AOP_CUDA_CHECK(cudaMallocHost(reinterpret_cast<void **>(&h_compact_), std::max<std::size_t>(1, n_) * sizeof(aop_accel::Contrib)));
    AOP_CUDA_CHECK(cudaMallocHost(reinterpret_cast<void **>(&h_counts_), (n_segments_ + 1) * sizeof(int)));
    AOP_CUDA_CHECK(cudaMallocHost(reinterpret_cast<void **>(&h_stage_), sizeof(aop_accel::Sensor) + (arcs_capacity_ + n_cond_ + 1) * 8));
  }

  ~CudaAccelerator() override
  {
    for (void * p : {static_cast<void *>(d_k_), static_cast<void *>(d_segs_), static_cast<void *>(d_elems_),
                     static_cast<void *>(d_vox_), static_cast<void *>(d_mem_), static_cast<void *>(d_sensor_),
                     static_cast<void *>(d_out_), static_cast<void *>(d_compact_), static_cast<void *>(d_flags_),
                     static_cast<void *>(d_offsets_), static_cast<void *>(d_counts_), static_cast<void *>(d_arc_begin_),
                     static_cast<void *>(d_arcs_), d_temp_}) {if (p) {cudaFree(p);}}
    if (h_compact_) {cudaFreeHost(h_compact_);}
    if (h_counts_) {cudaFreeHost(h_counts_);}
    if (h_stage_) {cudaFreeHost(h_stage_);}
    if (stream_) {cudaStreamDestroy(stream_);}
  }

  std::string name() const override {return "cuda";}

  bool evaluate(
    const SensorState & s, std::uint64_t scan_sequence, const std::vector<std::vector<double>> & arcs,
    std::vector<std::vector<Contribution>> & diffuse_out, std::vector<Contribution> & voxel_out,
    std::vector<Contribution> & member_out) override
  {
    try
    {
      if (arcs.size() != n_cond_) {return false;}
      std::vector<int> arc_begin{0};
      std::vector<double> flat;
      for (const auto & a : arcs) {flat.insert(flat.end(), a.begin(), a.end()); arc_begin.push_back(static_cast<int>(flat.size()));}
      if (flat.size() > arcs_capacity_)
      {
        cudaFree(d_arcs_); arcs_capacity_ = flat.size() * 2;
        AOP_CUDA_CHECK(cudaMalloc(&d_arcs_, arcs_capacity_ * sizeof(double)));
      }
      const auto sensor = make_sensor(s);
      AOP_CUDA_CHECK(cudaMemcpyAsync(d_sensor_, &sensor, sizeof(sensor), cudaMemcpyHostToDevice, stream_));
      AOP_CUDA_CHECK(cudaMemcpyAsync(d_arc_begin_, arc_begin.data(), arc_begin.size() * sizeof(int), cudaMemcpyHostToDevice, stream_));
      if (!flat.empty()) {AOP_CUDA_CHECK(cudaMemcpyAsync(d_arcs_, flat.data(), flat.size() * sizeof(double), cudaMemcpyHostToDevice, stream_));}
      const unsigned threads = 128, blocks = static_cast<unsigned>((n_ + threads - 1) / threads);
      evaluate_kernel<<<blocks, threads, 0, stream_>>>(d_k_, d_sensor_, scan_sequence, d_segs_, d_elems_, n_elems_, d_vox_, n_vox_,
        d_mem_, n_mem_, d_arcs_, d_arc_begin_, d_out_, d_flags_);
      AOP_CUDA_CHECK(cudaGetLastError());
      std::size_t tb = temp_bytes_;
      AOP_CUDA_CHECK(cub::DeviceSegmentedReduce::Sum(d_temp_, tb, d_flags_, d_counts_, n_segments_, d_offsets_, d_offsets_ + 1, stream_));
      tb = temp_bytes_;
      AOP_CUDA_CHECK(cub::DeviceSelect::Flagged(d_temp_, tb, d_out_, d_flags_, d_compact_, d_counts_ + n_segments_, static_cast<int>(n_), stream_));
      AOP_CUDA_CHECK(cudaMemcpyAsync(h_counts_, d_counts_, (n_segments_ + 1) * sizeof(int), cudaMemcpyDeviceToHost, stream_));
      AOP_CUDA_CHECK(cudaStreamSynchronize(stream_));
      const int total = h_counts_[n_segments_];
      if (total > 0)
      {
        AOP_CUDA_CHECK(cudaMemcpyAsync(h_compact_, d_compact_, static_cast<std::size_t>(total) * sizeof(aop_accel::Contrib), cudaMemcpyDeviceToHost, stream_));
        AOP_CUDA_CHECK(cudaStreamSynchronize(stream_));
      }
      diffuse_out.assign(n_cond_, {});
      std::size_t pos = 0;
      auto take = [&](std::vector<Contribution> & dst, int count) {
          dst.resize(static_cast<std::size_t>(count));
          if (count > 0) {std::memcpy(static_cast<void *>(dst.data()), h_compact_ + pos, static_cast<std::size_t>(count) * sizeof(Contribution));}
          pos += static_cast<std::size_t>(count);
        };
      for (std::size_t ci = 0; ci < n_cond_; ++ci) {take(diffuse_out[ci], h_counts_[ci]);}
      take(voxel_out, h_counts_[n_cond_]);
      take(member_out, h_counts_[n_cond_ + 1]);
      return pos == static_cast<std::size_t>(total);
    }
    catch (const std::exception &)
    {
      return false;   // the model falls back to the CPU reference path for this scan
    }
  }

private:
  AccelScene a_;
  std::uint32_t n_elems_{0}, n_vox_{0}, n_mem_{0}, n_{0};
  std::size_t n_cond_{0};
  int n_segments_{0};
  cudaStream_t stream_{nullptr};
  aop_accel::Consts * d_k_{nullptr};
  aop_accel::Seg * d_segs_{nullptr};
  aop_accel::Elem * d_elems_{nullptr};
  aop_accel::Vox * d_vox_{nullptr};
  aop_accel::Mem * d_mem_{nullptr};
  aop_accel::Sensor * d_sensor_{nullptr};
  aop_accel::Contrib * d_out_{nullptr};
  aop_accel::Contrib * d_compact_{nullptr};
  int * d_flags_{nullptr};
  int * d_offsets_{nullptr};
  int * d_counts_{nullptr};
  int * d_arc_begin_{nullptr};
  double * d_arcs_{nullptr};
  std::size_t arcs_capacity_{0};
  void * d_temp_{nullptr};
  std::size_t temp_bytes_{0};
  aop_accel::Contrib * h_compact_{nullptr};
  int * h_counts_{nullptr};
  unsigned char * h_stage_{nullptr};
};

}  // namespace

std::shared_ptr<CandidateAccelerator> make_cuda_accelerator(const SceneExport & e, std::string * error)
{
  try
  {
    int n = 0;
    if (cudaGetDeviceCount(&n) != cudaSuccess || n == 0) {if (error) {*error = "no CUDA device";} return nullptr;}
    return std::make_shared<CudaAccelerator>(e);
  }
  catch (const std::exception & ex)
  {
    if (error) {*error = ex.what();}
    return nullptr;
  }
}

}  // namespace aop_radar
