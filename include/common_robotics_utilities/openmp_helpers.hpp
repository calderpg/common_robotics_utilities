#pragma once

#include <cstdint>
#include <stdexcept>

#if defined(_OPENMP)
#include <omp.h>
#endif

namespace common_robotics_utilities
{
namespace openmp_helpers
{
/// Returns true if OpenMP is enabled in the build, false otherwise.
constexpr bool IsOmpEnabledInBuild()
{
#if defined(_OPENMP)
  return true;
#else
  return false;
#endif
}

/// Returns true if called from within an OpenMP context, false otherwise.
inline bool IsOmpInParallel()
{
#if defined(_OPENMP)
  return static_cast<bool>(omp_in_parallel());
#else
  return false;
#endif
}

/// Returns the OpenMP thread number in the current OpenMP parallel context.
/// If called from outside an OpenMP parallel context or without OpenMP enabled,
/// it will return 0.
inline int32_t GetContextOmpThreadNum()
{
#if defined(_OPENMP)
  return static_cast<int32_t>(omp_get_thread_num());
#else
  return 0;
#endif
}

/// Returns the maximum number of OpenMP threads available in the current OpenMP
/// parallel context. If called outside an OpenMP parallel context or without
/// OpenMP enabled, it will return 1.
inline int32_t GetContextMaxNumOmpThreads()
{
#if defined(_OPENMP)
  const int32_t max_threads = static_cast<int32_t>(omp_get_max_threads());
  if (max_threads > 0)
  {
    return max_threads;
  }
  else
  {
    throw std::runtime_error("OpenMP max threads <= 0");
  }
#else
  return 1;
#endif
}

/// Returns the current number of OpenMP threads available in the current OpenMP
/// parallel context. If called outside an OpenMP parallel context or without
/// OpenMP enabled, it will return 1.
inline int32_t GetContextNumOmpThreads()
{
#if defined(_OPENMP)
  const int32_t num_threads = static_cast<int32_t>(omp_get_num_threads());
  if (num_threads > 0)
  {
    return num_threads;
  }
  else
  {
    throw std::runtime_error("OpenMP num threads <= 0");
  }
#else
  return 1;
#endif
}

/// Returns the maximum number of OpenMP threads available in an OpenMP parallel
/// context. If called without OpenMP enabled, it will return 1.
/// Use caution if calling from within an existing OpenMP context, since nested
/// contexts are not enabled by default.
inline int32_t GetMaxNumOmpThreads()
{
#if defined(_OPENMP)
  int32_t max_num_threads = 0;
#pragma omp parallel
  {
    max_num_threads = GetContextMaxNumOmpThreads();
  }
  return max_num_threads;
#else
  return 1;
#endif
}

/// Returns the current number of OpenMP threads available in an OpenMP parallel
/// context. If called without OpenMP enabled, it will return 1.
/// Use caution if calling from within an existing OpenMP context, since nested
/// contexts are not enabled by default.
inline int32_t GetNumOmpThreads()
{
#if defined(_OPENMP)
  int32_t num_threads = 0;
#pragma omp parallel
  {
    num_threads = GetContextNumOmpThreads();
  }
  return num_threads;
#else
  return 1;
#endif
}

/// Returns the maximum possible number of OpenMP threads in the program. If
/// called without OpenMP enabled, it will return 1.
inline int32_t GetOmpThreadLimit()
{
#if defined(_OPENMP)
  const int32_t thread_limit = static_cast<int32_t>(omp_get_thread_limit());
  if (thread_limit > 0)
  {
    return thread_limit;
  }
  else
  {
    throw std::runtime_error("OpenMP thread limit <= 0");
  }
#else
  return 1;
#endif
}

/// RAII wrapper for changing the number of OpenMP threads temporarily.
/// This wrapper changes OpenMP settings for *all* of the current program!
/// It will throw an exception if called from within an existing OpenMP parallel
/// context.
class ChangeOmpNumThreadsWrapper
{
public:
  explicit ChangeOmpNumThreadsWrapper(const int32_t num_threads)
  {
    if (IsOmpInParallel())
    {
      throw std::runtime_error(
          "Cannot create ChangeOmpNumThreadsWrapper inside an OpenMP parallel "
          "context");
    }
    if (num_threads <= 0)
    {
      throw std::invalid_argument("num_threads must be greater than zero");
    }
    starting_num_omp_threads_ = GetNumOmpThreads();
#if defined(_OPENMP)
    omp_set_num_threads(num_threads);
#endif
  }

  virtual ~ChangeOmpNumThreadsWrapper()
  {
#if defined(_OPENMP)
    omp_set_num_threads(starting_num_omp_threads_);
#endif
  }

private:
  int32_t starting_num_omp_threads_ = 0;
};

/// RAII wrapper for disabling OpenMP temporarily.
/// This wrapper changes OpenMP settings for *all* of the current program!
/// It will throw an exception if called from within an existing OpenMP parallel
/// context.
class DisableOmpWrapper : public ChangeOmpNumThreadsWrapper
{
public:
  DisableOmpWrapper() : ChangeOmpNumThreadsWrapper(1) {}
};

/// Macro to stringify tokens for the purposes of the below macros.
#define CRU_MACRO_STRINGIFY(s) #s

/// Macros to declare OpenMP parallel for loops, handling conditionals as well
/// as the case of OpenMP being disabled entirely.
#if defined(_OPENMP)
#define CRU_OMP_PARALLEL_FOR_IF(enable_parallel) _Pragma(CRU_MACRO_STRINGIFY(omp parallel for if(enable_parallel)))
#define CRU_OMP_PARALLEL_FOR _Pragma(CRU_MACRO_STRINGIFY(omp parallel for))
#else
#define CRU_OMP_PARALLEL_FOR_IF(enable_parallel) (void)(enable_parallel);
#define CRU_OMP_PARALLEL_FOR
#endif
}  // namespace openmp_helpers
}  // namespace common_robotics_utilities

