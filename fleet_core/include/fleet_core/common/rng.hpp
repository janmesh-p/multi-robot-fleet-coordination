#pragma once

#include <array>
#include <cstdint>
#include <string_view>

namespace fleet_core {

// Portable deterministic random numbers.
//
// std::poisson_distribution and friends are implementation-defined, so one seed can give
// different draws on GCC 11, GCC 13, or libc++. Every algorithm here is fully specified,
// so a seed replays the same run on the desktop, the Jetson, and CI.
class Rng {
 public:
  explicit Rng(uint64_t seed);

  // xoshiro256** raw output.
  uint64_t next();

  // Uniform in [0, 1) with 53-bit resolution.
  double uniform01();

  // Uniform in [lo, hi], unbiased. Throws std::invalid_argument if lo > hi.
  int64_t uniformInt(int64_t lo, int64_t hi);

  // True with probability p. Always consumes exactly one draw, so runs that differ only in p
  // share their random numbers and sweep curves stay smooth.
  bool bernoulli(double p);

  // Poisson count with mean lambda in [0, 1e6]. Throws std::invalid_argument otherwise.
  int32_t poisson(double lambda);

 private:
  int32_t poissonKnuth(double lambda);

  std::array<uint64_t, 4> s_{};
};

// Seed for one named random stream. Each subsystem draws from its own stream, so a change in
// one never shifts another: every planner compared under one master seed sees the same orders.
uint64_t deriveSeed(uint64_t master_seed, std::string_view stream);

namespace rng_stream {
inline constexpr std::string_view kSpawn = "spawn";
inline constexpr std::string_view kOrders = "orders";
inline constexpr std::string_view kDelays = "delays";
inline constexpr std::string_view kLink = "link";
}  // namespace rng_stream

}  // namespace fleet_core
