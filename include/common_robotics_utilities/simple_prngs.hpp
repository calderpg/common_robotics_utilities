#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

namespace common_robotics_utilities
{
namespace simple_prngs
{
/// Implementation of the "Split-Mix 64" PRNG.
class SplitMix64PRNG
{
private:

  uint64_t state_ = 0;

  uint64_t Next()
  {
    uint64_t z = (state_ += UINT64_C(0x9E3779B97F4A7C15));
    z = (z ^ (z >> 30)) * UINT64_C(0xBF58476D1CE4E5B9);
    z = (z ^ (z >> 27)) * UINT64_C(0x94D049BB133111EB);
    return z ^ (z >> 31);
  }

public:

  explicit SplitMix64PRNG(const uint64_t seed_val)
  {
    seed(seed_val);
  }

  static constexpr uint64_t min(void)
  {
    return 0u;
  }

  static constexpr uint64_t max(void)
  {
    return std::numeric_limits<uint64_t>::max();
  }

  void seed(const uint64_t seed_val)
  {
    state_ = seed_val;
  }

  void discard(const uint64_t z)
  {
    // This suppresses "set but not used" warnings
    uint64_t temp __attribute__((unused));
    temp = 0u;
    for (uint64_t i = 0; i < z; i++)
    {
      // This should prevent the compiler from optimizing out the loop
      temp = Next();
      __asm__ __volatile__("");
    }
  }

  uint64_t operator() (void)
  {
    return Next();
  }
};

/// Implementation of the "XOR-Shift-128-Plus" PRNG.
class XorShift128PlusPRNG
{
private:

  uint64_t state_1_ = 0;
  uint64_t state_2_ = 0;

  uint64_t Next()
  {
    uint64_t s1 = state_1_;
    const uint64_t s0 = state_2_;
    state_1_ = s0;
    s1 ^= s1 << 23; // a
    state_2_ = s1 ^ s0 ^ (s1 >> 18) ^ (s0 >> 5); // b, c
    return state_2_ + s0;
  }

public:

  explicit XorShift128PlusPRNG(const uint64_t seed_val)
  {
    seed(seed_val);
  }

  static constexpr uint64_t min(void)
  {
    return 0u;
  }

  static constexpr uint64_t max(void)
  {
    return std::numeric_limits<uint64_t>::max();
  }

  void seed(const uint64_t seed_val)
  {
    SplitMix64PRNG temp_seed_gen(seed_val);
    state_1_ = temp_seed_gen();
    state_2_ = temp_seed_gen();
  }

  void discard(const uint64_t z)
  {
    // This suppresses "set but not used" warnings
    uint64_t temp __attribute__((unused));
    temp = 0u;
    for (uint64_t i = 0; i < z; i++)
    {
      temp = Next();
      // This should prevent the compiler from optimizing out the loop
      __asm__ __volatile__("");
    }
  }

  inline uint64_t operator() (void)
  {
    return Next();
  }
};

/// Implementation of the "XOR-Shift-1024-Star" PRNG.
class XorShift1024StarPRNG
{
private:

  std::array<uint64_t, 16> state_ = {{0, 0, 0, 0, 0, 0, 0, 0,
                                      0, 0, 0, 0, 0, 0, 0, 0}};
  size_t p_ = 0;

  uint64_t Next()
  {
    const uint64_t s0 = state_.at(p_);
    p_ = (p_ + 1) & 15;
    uint64_t s1 = state_.at(p_);
    s1 ^= s1 << 31; // a
    state_.at(p_) = s1 ^ s0 ^ (s1 >> 11) ^ (s0 >> 30); // b,c
    return state_.at(p_) * UINT64_C(1181783497276652981);
  }

public:

  explicit XorShift1024StarPRNG(const uint64_t seed_val)
  {
    seed(seed_val);
    p_ = 0;
  }

  static constexpr uint64_t min(void)
  {
    return 0u;
  }

  static constexpr uint64_t max(void)
  {
    return std::numeric_limits<uint64_t>::max();
  }

  void seed(const uint64_t seed_val)
  {
    SplitMix64PRNG temp_seed_gen(seed_val);
    for (size_t idx = 0u; idx < state_.size(); idx++)
    {
      state_[idx] = temp_seed_gen();
    }
  }

  void discard(const uint64_t z)
  {
    // This suppresses "set but not used" warnings
    uint64_t temp __attribute__((unused));
    temp = 0u;
    for (uint64_t i = 0; i < z; i++)
    {
      temp = Next();
      // This should prevent the compiler from optimizing out the loop
      __asm__ __volatile__("");
    }
  }

  uint64_t operator() (void)
  {
    return Next();
  }
};
}  // namespace simple_prngs
}  // namespace common_robotics_utilities
