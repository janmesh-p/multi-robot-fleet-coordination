#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include "fleet_core/common/rng.hpp"

namespace fleet_core {
namespace {

// Pinned outputs, cross-checked against an independent Python implementation of splitmix64,
// xoshiro256**, and FNV-1a. If these change, every recorded seed silently means a different
// run, so only change the algorithms on purpose.
TEST(Rng, GoldenRawSequence) {
  Rng rng(42);
  EXPECT_EQ(rng.next(), 0x15780B2E0C2EC716ULL);
  EXPECT_EQ(rng.next(), 0x6104D9866D113A7EULL);
  EXPECT_EQ(rng.next(), 0xAE17533239E499A1ULL);
  EXPECT_EQ(rng.next(), 0xECB8AD4703B360A1ULL);
}

TEST(Rng, GoldenDerivedValues) {
  EXPECT_EQ(deriveSeed(1, "orders"), 0x655D4BCAA6D685C9ULL);
  EXPECT_EQ(deriveSeed(1, "delays"), 0x08683207B295A62DULL);

  Rng ints(42);
  const std::array<int64_t, 6> expected_ints{42, 2, 9, 93, 76, 84};
  for (const int64_t v : expected_ints) EXPECT_EQ(ints.uniformInt(0, 99), v);

  Rng floats(42);
  EXPECT_EQ(floats.uniform01(), 0.083862971059882163);
  EXPECT_EQ(floats.uniform01(), 0.37898025066266861);
}

TEST(Rng, SameSeedSameSequence) {
  Rng a(7);
  Rng b(7);
  for (int i = 0; i < 1000; ++i) ASSERT_EQ(a.next(), b.next());
}

TEST(Rng, StreamsAreIndependent) {
  EXPECT_NE(deriveSeed(7, rng_stream::kOrders), deriveSeed(7, rng_stream::kDelays));
  EXPECT_NE(deriveSeed(7, rng_stream::kLink), deriveSeed(7, rng_stream::kSpawn));
  EXPECT_NE(deriveSeed(7, rng_stream::kOrders), deriveSeed(8, rng_stream::kOrders));
  EXPECT_EQ(deriveSeed(7, rng_stream::kOrders), deriveSeed(7, "orders"));
}

TEST(Rng, Uniform01StaysInRange) {
  Rng rng(1);
  for (int i = 0; i < 100000; ++i) {
    const double u = rng.uniform01();
    ASSERT_GE(u, 0.0);
    ASSERT_LT(u, 1.0);
  }
}

TEST(Rng, UniformIntCoversRangeEvenly) {
  Rng rng(3);
  constexpr int kDraws = 100000;
  std::array<int, 10> counts{};
  for (int i = 0; i < kDraws; ++i) {
    const int64_t v = rng.uniformInt(0, 9);
    ASSERT_GE(v, 0);
    ASSERT_LE(v, 9);
    ++counts[static_cast<size_t>(v)];
  }
  for (const int count : counts) EXPECT_NEAR(count, kDraws / 10, 500);  // about 5 sigma
}

TEST(Rng, UniformIntEdgeCases) {
  Rng rng(4);
  EXPECT_EQ(rng.uniformInt(5, 5), 5);
  EXPECT_EQ(rng.uniformInt(-3, -3), -3);
  for (int i = 0; i < 1000; ++i) {
    const int64_t v = rng.uniformInt(-2, 2);
    ASSERT_GE(v, -2);
    ASSERT_LE(v, 2);
  }
  const int64_t lo = std::numeric_limits<int64_t>::min();
  const int64_t hi = std::numeric_limits<int64_t>::max();
  EXPECT_NO_THROW(rng.uniformInt(lo, hi));
  EXPECT_THROW(rng.uniformInt(2, 1), std::invalid_argument);
}

TEST(Rng, BernoulliFrequencyAndEdges) {
  Rng rng(5);
  int hits = 0;
  for (int i = 0; i < 100000; ++i) hits += rng.bernoulli(0.3) ? 1 : 0;
  EXPECT_NEAR(hits, 30000, 750);  // about 5 sigma
  for (int i = 0; i < 1000; ++i) {
    ASSERT_FALSE(rng.bernoulli(0.0));
    ASSERT_TRUE(rng.bernoulli(1.0));
  }
}

TEST(Rng, BernoulliAlwaysConsumesOneDraw) {
  Rng a(9);
  Rng b(9);
  (void)a.bernoulli(0.0);
  (void)b.bernoulli(0.7);
  EXPECT_EQ(a.next(), b.next());
}

TEST(Rng, PoissonMeanAndVariance) {
  for (const double lambda : {0.2, 3.0, 75.0}) {  // 75 exercises the chunked path
    Rng rng(6);
    constexpr int kDraws = 200000;
    double sum = 0.0;
    double sum_sq = 0.0;
    for (int i = 0; i < kDraws; ++i) {
      const double k = rng.poisson(lambda);
      sum += k;
      sum_sq += k * k;
    }
    const double mean = sum / kDraws;
    const double variance = sum_sq / kDraws - mean * mean;
    EXPECT_NEAR(mean, lambda, 5.0 * std::sqrt(lambda / kDraws)) << "lambda " << lambda;
    EXPECT_NEAR(variance, lambda, 0.03 * lambda + 0.01) << "lambda " << lambda;
  }
}

TEST(Rng, PoissonEdgeCases) {
  Rng rng(7);
  for (int i = 0; i < 100; ++i) ASSERT_EQ(rng.poisson(0.0), 0);
  EXPECT_THROW(rng.poisson(-1.0), std::invalid_argument);
  EXPECT_THROW(rng.poisson(std::numeric_limits<double>::quiet_NaN()), std::invalid_argument);
  EXPECT_THROW(rng.poisson(std::numeric_limits<double>::infinity()), std::invalid_argument);
}

}  // namespace
}  // namespace fleet_core
