// What a profile's timestamp MEANS.
//
// Stamping a profile with node->now() from inside
// unpackProfileToPointCloud() records the moment the driver reached that
// line: after the TCP round trip, after the decode, after whatever the ROS
// executor was doing first. Downstream, tari_pc_reconstruction asks TF where
// the sensor was at that stamp, so every millisecond of driver latency becomes
// a millimetre of arm travel baked into the geometry at 1 m/s, silently and
// with no symptom beyond a slightly wrong part.
//
// These are the rules the acquisition-time estimate obeys. They are pure
// arithmetic on
// nanoseconds, which is the whole reason the calculation lives in
// keyence/impl/profile_timing.h instead of inline in the acquisition loop:
// a stamp derived from a clock read cannot be tested, and this one can.

#include <gtest/gtest.h>

#include <cstdint>

#include <keyence/impl/profile_timing.h>

namespace
{
constexpr int64_t kMs = 1000000;          // one millisecond in nanoseconds
constexpr int64_t kMaxBackdate = 50 * kMs;  // the driver's default clamp
}  // namespace

// The headline: a profile is stamped BEFORE the reply that carried it.
TEST(ProfileTiming, StampPrecedesTheReply)
{
  const int64_t t_request = 1000 * kMs;
  const int64_t t_reply = 1004 * kMs;              // a 4 ms round trip
  const int64_t sampling_period = kMs;             // a 1000 Hz program

  const int64_t stamp = keyence::profileAcquisitionTimeNs(t_request, t_reply, sampling_period,
                                                          kMaxBackdate);

  // half the round trip (2 ms) plus half the sampling period (0.5 ms)
  EXPECT_EQ(stamp, t_reply - 2 * kMs - kMs / 2);
  EXPECT_LT(stamp, t_reply) << "a measurement cannot postdate its own reply";
  EXPECT_GT(stamp, t_request - kMaxBackdate);
}

// THE REGRESSION. The old behaviour was stamp == the clock at publish time,
// which is at or after t_reply. Nothing this function returns can equal that
// for a round trip of non-zero length, which is what makes the two
// distinguishable from outside the driver.
TEST(ProfileTiming, NeverStampsAtPublishTime)
{
  for (int64_t round_trip_ms = 1; round_trip_ms <= 20; ++round_trip_ms)
  {
    const int64_t t_request = 5000 * kMs;
    const int64_t t_reply = t_request + round_trip_ms * kMs;
    const int64_t stamp =
        keyence::profileAcquisitionTimeNs(t_request, t_reply, kMs, kMaxBackdate);
    EXPECT_LT(stamp, t_reply) << "round trip " << round_trip_ms << " ms";
    // and it is a real correction, not a token one: at least half the round trip
    EXPECT_LE(stamp, t_reply - (round_trip_ms * kMs) / 2);
  }
}

// A slow controller program means a wider acquisition window, and the stamp
// moves accordingly. This is the "half a sampling period" term on its own.
TEST(ProfileTiming, SlowerProgramBackdatesFurther)
{
  const int64_t t_request = 2000 * kMs;
  const int64_t t_reply = t_request;  // isolate the sampling-period term

  const int64_t fast = keyence::profileAcquisitionTimeNs(t_request, t_reply, kMs, kMaxBackdate);
  const int64_t slow =
      keyence::profileAcquisitionTimeNs(t_request, t_reply, 10 * kMs, kMaxBackdate);

  EXPECT_EQ(t_reply - fast, kMs / 2);
  EXPECT_EQ(t_reply - slow, 5 * kMs);
  EXPECT_LT(slow, fast);
}

// An unknown sampling rate drops that half of the correction rather than
// guessing. The round-trip half still applies.
TEST(ProfileTiming, UnknownSamplingRateUsesRoundTripOnly)
{
  const int64_t t_request = 7000 * kMs;
  const int64_t t_reply = t_request + 6 * kMs;

  EXPECT_EQ(keyence::profileAcquisitionTimeNs(t_request, t_reply, 0, kMaxBackdate),
            t_reply - 3 * kMs);
}

// THE CLAMP. A stalled round trip -- a reconnect, a scheduling hiccup, a
// controller that took a second to answer -- must not backdate a profile
// arbitrarily far: TF will happily interpolate to wherever the arm was half a
// second ago and place the profile there.
TEST(ProfileTiming, StallDoesNotBackdateArbitrarily)
{
  const int64_t t_request = 0;
  const int64_t t_reply = 2000 * kMs;  // a two-second round trip

  const int64_t stamp = keyence::profileAcquisitionTimeNs(t_request, t_reply, kMs, kMaxBackdate);

  EXPECT_EQ(stamp, t_reply - kMaxBackdate);
}

// A clock that jumped backwards between the two reads (sim time, an NTP step)
// must not produce a NEGATIVE correction, which would stamp the profile in the
// future and make every TF lookup wait for data that does not exist yet.
TEST(ProfileTiming, ReversedClockDoesNotPostdate)
{
  const int64_t t_request = 9000 * kMs;
  const int64_t t_reply = 8000 * kMs;  // earlier than the request

  const int64_t stamp = keyence::profileAcquisitionTimeNs(t_request, t_reply, kMs, kMaxBackdate);

  EXPECT_LE(stamp, t_reply);
  EXPECT_EQ(stamp, t_reply - kMs / 2) << "round-trip term dropped, sampling term kept";
}

// Degenerate inputs are absorbed rather than propagated.
TEST(ProfileTiming, DegenerateInputs)
{
  const int64_t t = 1234 * kMs;
  // no clamp allowed at all: the stamp is the reply time
  EXPECT_EQ(keyence::profileAcquisitionTimeNs(t - kMs, t, kMs, 0), t);
  // a negative clamp is treated as zero, not as a licence to postdate
  EXPECT_EQ(keyence::profileAcquisitionTimeNs(t - kMs, t, kMs, -5 * kMs), t);
  // a negative sampling period contributes nothing
  EXPECT_EQ(keyence::profileAcquisitionTimeNs(t, t, -kMs, kMaxBackdate), t);
}

// The measured-latency path, for a cell that has pinned the number.
TEST(ProfileTiming, FixedLatencyIsUsedVerbatimAndClamped)
{
  const int64_t t_reply = 4000 * kMs;

  EXPECT_EQ(keyence::profileAcquisitionTimeNsFixed(t_reply, 3 * kMs, kMaxBackdate),
            t_reply - 3 * kMs);
  // above the clamp it is clamped ...
  EXPECT_EQ(keyence::profileAcquisitionTimeNsFixed(t_reply, 900 * kMs, kMaxBackdate),
            t_reply - kMaxBackdate);
  // ... and a negative latency cannot push the stamp into the future
  EXPECT_EQ(keyence::profileAcquisitionTimeNsFixed(t_reply, -7 * kMs, kMaxBackdate), t_reply);
}

// Monotonic in every input that should be, which is what keeps a sequence of
// profiles in the order they were measured.
TEST(ProfileTiming, StampsFollowReplyOrder)
{
  int64_t previous = 0;
  for (int i = 0; i < 100; ++i)
  {
    // 222 Hz replies, 4.5 ms apart, each with a 4 ms round trip
    const int64_t t_reply = 1000 * kMs + static_cast<int64_t>(i * 4.5 * kMs);
    const int64_t t_request = t_reply - 4 * kMs;
    const int64_t stamp =
        keyence::profileAcquisitionTimeNs(t_request, t_reply, kMs, kMaxBackdate);
    EXPECT_GT(stamp, previous);
    previous = stamp;
  }
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
