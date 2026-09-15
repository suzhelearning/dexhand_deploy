#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <limits>

namespace tianji_control {
using Joints7 = std::array<double, 7>;
struct CommandConfig {
  double maximum_step, time_window, rate, proposal_timeout;
};
enum class CommandError { ok, hard_limits, timestamp_rollback, source_stale,
                          maximum_step, tracking_hold_real };
struct Validation {
  CommandError error = CommandError::ok;
  double delta = 0, allowed = 0, elapsed = 0;
};
inline Validation validate_command(const Joints7& q, const Joints7& old,
    const Joints7& lower, const Joints7& upper, const CommandConfig& config,
    std::int64_t now, std::int64_t source, std::optional<std::int64_t> anchor,
    bool tracking_hold, bool simulation) {
  Validation out;
  if (tracking_hold && !simulation) { out.error = CommandError::tracking_hold_real; return out; }
  for (size_t i = 0; i < q.size(); ++i)
    if (!std::isfinite(q[i]) || !(lower[i] <= q[i] && q[i] <= upper[i])) {
      out.error = CommandError::hard_limits; return out;
    }
  out.allowed = 2.0 * config.maximum_step;
  if (config.time_window > 0) {
    if (anchor) {
      out.elapsed = static_cast<double>(source - *anchor) / 1e9;
      if (out.elapsed < 0) { out.error = CommandError::timestamp_rollback; return out; }
      const double speed = config.maximum_step * config.rate;
      out.allowed = std::max(out.allowed, speed * std::min(out.elapsed, config.time_window));
    }
    const double timeout_ns = config.proposal_timeout * 1e9;
    // Python int-to-float comparisons do not first round the integer to
    // double. Compare to the floored positive threshold to retain that rule.
    const bool age_expired = now >= source &&
      timeout_ns < static_cast<double>(std::numeric_limits<std::int64_t>::max()) &&
      now - source > static_cast<std::int64_t>(timeout_ns);
    if (now < source || age_expired) {
      out.error = CommandError::source_stale; return out;
    }
  }
  if (tracking_hold) return out;
  for (size_t i = 0; i < q.size(); ++i) out.delta = std::max(out.delta, std::abs(q[i] - old[i]));
  if (out.delta > out.allowed + (config.time_window > 0 ? 1e-10 : 0.0))
    out.error = CommandError::maximum_step;
  return out;
}
inline Joints7 track_command(const Joints7& target, const Joints7& old,
    double step, bool clipping, bool tracking_hold) {
  if (tracking_hold) return old;
  if (!clipping) return target;
  Joints7 result;
  for (size_t i = 0; i < old.size(); ++i)
    result[i] = old[i] + std::max(-step, std::min(step, target[i] - old[i]));
  return result;
}
inline Joints7 home_command(const Joints7& start, const Joints7& home,
    double elapsed, double minimum_duration, double maximum_speed) {
  double distance = 0;
  for (size_t i = 0; i < start.size(); ++i) distance = std::max(distance, std::abs(start[i] - home[i]));
  const double duration = std::max(minimum_duration, distance / maximum_speed);
  const double fraction = std::min(1., std::max(0., elapsed) / duration);
  if (fraction >= 1.) return home;  // Exact Home, not an interpolated approximation.
  Joints7 result;
  for (size_t i = 0; i < start.size(); ++i) result[i] = start[i] + fraction * (home[i] - start[i]);
  return result;
}
}  // namespace tianji_control
