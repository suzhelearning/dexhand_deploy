// Single-owner execution receipt bookkeeping. No robot/network authority.
#pragma once
#include <algorithm>
#include <array>
#include <cstdint>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>

namespace tianji_control {
using Positions = std::array<double, 14>;
struct Pending { std::int64_t sent; Positions positions; };
struct StateError : std::runtime_error {
  std::string detail;
  explicit StateError(const std::string& value) : std::runtime_error(value), detail(value) {}
};
class ExecutionGuard {
 public:
  ExecutionGuard(std::int64_t age, std::int64_t capacity) : age_(age), capacity_(capacity) {
    if (age <= 0 || capacity <= 0) throw std::invalid_argument("positive execution limits required");
  }
  void pause(const std::string& reason) {
    if (reason.empty()) throw std::invalid_argument("pause reason required");
    if (!reason_) reason_ = reason;
    pending_.clear();
  }
  bool check(std::int64_t now) {
    if (now <= 0) throw std::invalid_argument("now_ns must be positive int64");
    if (now < last_now_) pause("execution clock rollback");
    last_now_ = std::max(last_now_, now);
    for (const auto& [tick, pending] : pending_) {
      (void)tick;
      if (now-pending.sent > age_) {
        pause("coordinator receipt timeout");
        break;
      }
    }
    return !reason_;
  }
  void validate_tick(std::int64_t tick) const {
    if (tick <= 0) throw std::invalid_argument("tick_id must be positive int64");
    if (tick-1 != last_tick_) throw std::invalid_argument("execution ticks must be consecutive");
  }
  void register_tick(std::int64_t tick, std::int64_t now, const Positions& positions) {
    validate_tick(tick);
    if (!check(now)) throw StateError(*reason_);
    if (pending_.size() >= static_cast<std::size_t>(capacity_)) {
      pause("execution in-flight limit exceeded");
      throw StateError(*reason_);
    }
    pending_.emplace(tick, Pending{now, positions});
    last_tick_ = tick;
  }
  const Pending* find(std::int64_t tick) const {
    const auto it = pending_.find(tick);
    return it == pending_.end() ? nullptr : &it->second;
  }
  bool observe(std::int64_t tick, bool accepted, const std::string& reason, const Positions& q) {
    const auto* pending = find(tick);
    if (!pending || reason_) return false;
    if (!accepted) { pause("coordinator rejected bilateral command: " + reason); return false; }
    if (q != pending->positions) { pause("reference_direct command was modified downstream"); return false; }
    pending_.erase(tick);
    return true;
  }
  const std::optional<std::string>& reason() const { return reason_; }
  std::size_t in_flight() const { return pending_.size(); }
 private:
  std::int64_t age_, capacity_, last_tick_{}, last_now_{};
  std::map<std::int64_t, Pending> pending_;
  std::optional<std::string> reason_;
};
} // namespace tianji_control
