#ifndef KEYENCE_PROFILE_TIMING_H
#define KEYENCE_PROFILE_TIMING_H

#include <cstdint>

namespace keyence
{

/**
 * @brief When was this profile actually measured?
 *
 * THE POINT OF THIS FILE. Everything downstream that places a profile in the
 * world does so by asking TF where the sensor was at the profile's stamp. A
 * stamp taken when the driver got round to publishing is therefore not a
 * timestamp, it is a report of the driver's own scheduling, and the geometry
 * it produces is wrong by (however late the driver was) x (however fast the
 * arm is moving). Stamping with node->now() inside
 * unpackProfileToPointCloud() does exactly that: it is after the round trip,
 * after the decode, and after however long the ROS executor took to get back
 * to the loop.
 *
 * THERE IS NO BATCHING TO UNPICK, and the naming is why one might think there
 * is. ProfileInformation::num_profiles is the number of POINTS ACROSS THE
 * LASER LINE in ONE profile -- 800 on an LJ-V7080 -- not a count of profiles.
 * SingleProfile::Request encodes a literal 1 as its profile count, the
 * decoder resizes profile_points to num_profiles, and the unpacker walks them
 * as x = x_start + i * x_increment, which is the X axis of a single line.
 * One profile per TCP round trip, and that round trip is the driver's rate
 * ceiling (136-219 Hz measured against a real LJ-V7080). So each reply needs
 * exactly one stamp, and it has to describe the measurement rather than the
 * publication.
 *
 * WHAT IS KNOWABLE. The controller free-runs at its program's sampling rate
 * and a profile request is answered with the most recently completed profile.
 * Two things put the measurement before the reply:
 *
 *   1. the round trip. The controller served the request at some instant
 *      between the request going out and the reply coming back; with no
 *      better information, the midpoint.
 *   2. the sampling period. The profile handed back finished somewhere in the
 *      one sampling period before that instant, uniformly; in expectation,
 *      half a period earlier.
 *
 * so
 *
 *      stamp = (t_request + t_reply)/2 - sampling_period/2
 *
 * This is an estimate and it is deliberately a modest one: at 222 Hz round
 * trips and a 1000 Hz program the correction is about 2.7 ms.
 * It is not a substitute for a hardware-synchronised clock,
 * and a cell that ever measures the true latency should pin it with the
 * driver's acquisition_latency parameter instead of trusting this model.
 */

/// Clamp helper kept out of the way; std::clamp would drag in <algorithm> for
/// a header this small.
inline int64_t clampInt64(int64_t value, int64_t low, int64_t high)
{
  if (value < low)
  {
    return low;
  }
  if (value > high)
  {
    return high;
  }
  return value;
}

/**
 * @brief The estimated acquisition instant of a profile, in nanoseconds.
 *
 * @param t_request_ns       ROS time captured immediately BEFORE the profile
 *                           request was written to the socket.
 * @param t_reply_ns         ROS time captured immediately AFTER the reply was
 *                           fully read. Must not precede @p t_request_ns.
 * @param sampling_period_ns The controller program's sampling period
 *                           (1 / its sampling rate). 0 when unknown, which
 *                           simply drops that half of the correction.
 * @param max_backdate_ns    Hard limit on how far before @p t_reply_ns the
 *                           stamp may be placed. A round trip that stalled --
 *                           a reconnect, a scheduling hiccup, a controller
 *                           that took a whole second to answer -- must not be
 *                           allowed to backdate a profile arbitrarily, since
 *                           half of a one-second round trip is 500 ms of
 *                           arm travel in the wrong direction. Must be >= 0.
 *
 * The result is always in [t_reply_ns - max_backdate_ns, t_reply_ns]: a
 * measurement cannot postdate the reply that carried it, and it cannot be
 * backdated past the clamp. A negative or reversed input pair is treated as a
 * zero-length round trip rather than producing nonsense.
 */
inline int64_t profileAcquisitionTimeNs(int64_t t_request_ns, int64_t t_reply_ns,
                                        int64_t sampling_period_ns, int64_t max_backdate_ns)
{
  if (max_backdate_ns < 0)
  {
    max_backdate_ns = 0;
  }
  if (sampling_period_ns < 0)
  {
    sampling_period_ns = 0;
  }
  // A reply that appears to precede its own request means the clock moved
  // (a sim-time jump, an NTP step). Fall back to "no round-trip correction"
  // rather than adding a negative one.
  const int64_t round_trip_ns = (t_reply_ns >= t_request_ns) ? (t_reply_ns - t_request_ns) : 0;

  const int64_t backdate_ns = round_trip_ns / 2 + sampling_period_ns / 2;
  return t_reply_ns - clampInt64(backdate_ns, 0, max_backdate_ns);
}

/**
 * @brief The same, with the latency pinned by measurement instead of modelled.
 *
 * For a cell that has measured how far behind the reply the acquisition
 * actually is. @p latency_ns is subtracted from @p t_reply_ns directly and is
 * still clamped by @p max_backdate_ns.
 */
inline int64_t profileAcquisitionTimeNsFixed(int64_t t_reply_ns, int64_t latency_ns,
                                             int64_t max_backdate_ns)
{
  if (max_backdate_ns < 0)
  {
    max_backdate_ns = 0;
  }
  return t_reply_ns - clampInt64(latency_ns, 0, max_backdate_ns);
}

}  // namespace keyence

#endif  // KEYENCE_PROFILE_TIMING_H
