#include "fleet_core/common/rng.hpp"

#include <cmath>
#include <stdexcept>

namespace fleet_core {
namespace {

// splitmix64 (Steele, Lea, Flood 2014). Expands one seed into well-mixed state words.
uint64_t splitmix64(uint64_t& state) {
  uint64_t z = (state += 0x9E3779B97F4A7C15ULL);
  z = (z ^ (z >> 30U)) * 0xBF58476D1CE4E5B9ULL;
  z = (z ^ (z >> 27U)) * 0x94D049BB133111EBULL;
  return z ^ (z >> 31U);
}

constexpr uint64_t rotl(uint64_t x, unsigned k) { return (x << k) | (x >> (64U - k)); }

}  // namespace

Rng::Rng(uint64_t seed) {
  uint64_t state = seed;
  for (uint64_t& word : s_) word = splitmix64(state);
}

// xoshiro256** 1.0 (Blackman, Vigna 2018).
uint64_t Rng::next() {
  const uint64_t result = rotl(s_[1] * 5U, 7U) * 9U;
  const uint64_t t = s_[1] << 17U;
  s_[2] ^= s_[0];
  s_[3] ^= s_[1];
  s_[1] ^= s_[2];
  s_[0] ^= s_[3];
  s_[2] ^= t;
  s_[3] = rotl(s_[3], 45U);
  return result;
}

double Rng::uniform01() {
  // Top 53 bits scaled by 2^-53: exact in double, never 1.0.
  return static_cast<double>(next() >> 11U) * 0x1.0p-53;
}

int64_t Rng::uniformInt(int64_t lo, int64_t hi) {
  if (lo > hi) throw std::invalid_argument("Rng::uniformInt: lo > hi");
  const uint64_t range = static_cast<uint64_t>(hi) - static_cast<uint64_t>(lo) + 1U;
  if (range == 0) return static_cast<int64_t>(next());  // full 64-bit range
  // Reject the lowest (2^64 mod range) outputs so every residue is equally likely.
  const uint64_t threshold = (uint64_t{0} - range) % range;
  uint64_t x = next();
  while (x < threshold) x = next();
  return static_cast<int64_t>(static_cast<uint64_t>(lo) + x % range);
}

bool Rng::bernoulli(double p) { return uniform01() < p; }

int32_t Rng::poisson(double lambda) {
  if (!std::isfinite(lambda) || lambda < 0.0 || lambda > 1e6) {
    throw std::invalid_argument("Rng::poisson: lambda must be in [0, 1e6]");
  }
  // A sum of independent Poisson counts is Poisson, so large means are split into chunks that
  // Knuth's method handles without underflow.
  constexpr double kChunk = 30.0;
  int32_t count = 0;
  while (lambda > kChunk) {
    count += poissonKnuth(kChunk);
    lambda -= kChunk;
  }
  return count + poissonKnuth(lambda);
}

int32_t Rng::poissonKnuth(double lambda) {
  const double limit = std::exp(-lambda);
  int32_t k = 0;
  double product = uniform01();
  while (product > limit) {
    ++k;
    product *= uniform01();
  }
  return k;
}

uint64_t deriveSeed(uint64_t master_seed, std::string_view stream) {
  uint64_t hash = 0xCBF29CE484222325ULL;  // FNV-1a 64-bit
  for (const char c : stream) {
    hash ^= static_cast<uint8_t>(c);
    hash *= 0x100000001B3ULL;
  }
  uint64_t state = master_seed ^ hash;
  return splitmix64(state);
}

}  // namespace fleet_core
