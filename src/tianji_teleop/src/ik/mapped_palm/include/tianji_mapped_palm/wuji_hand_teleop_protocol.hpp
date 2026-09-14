#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace tianji_mapped_palm {

inline constexpr std::size_t kWujiHandJointDof = 20U;
inline constexpr std::size_t kWujiHandTeleopPacketSize = 348U;
inline constexpr std::uint8_t kWujiHandPacketVersion = 1U;
inline constexpr std::uint8_t kWujiHandLeftValidFlag = 1U << 0U;
inline constexpr std::uint8_t kWujiHandRightValidFlag = 1U << 1U;
inline constexpr std::uint8_t kWujiHandKnownFlags =
    kWujiHandLeftValidFlag | kWujiHandRightValidFlag;

struct WujiHandTeleopFrame {
  std::uint64_t sequence{0U};
  std::int64_t source_timestamp_ns{0};
  std::array<double, kWujiHandJointDof> left{};
  std::array<double, kWujiHandJointDof> right{};
  bool left_valid{false};
  bool right_valid{false};
  // Receiver-local metadata; this field is never encoded on the wire.
  std::int64_t receive_monotonic_ns{0};
};

enum class WujiHandPacketError {
  kNone,
  kWrongSize,
  kWrongMagic,
  kWrongVersion,
  kWrongDeclaredSize,
  kInvalidFlags,
  kCrcMismatch,
  kInvalidMetadata,
  kNonFiniteJointPosition,
};

struct WujiHandPacketDecodeResult {
  WujiHandPacketError error{WujiHandPacketError::kWrongSize};
  std::optional<WujiHandTeleopFrame> frame;
};

std::vector<std::uint8_t> encodeWujiHandTeleopPacket(
    const WujiHandTeleopFrame& frame);

WujiHandPacketDecodeResult decodeWujiHandTeleopPacket(
    const std::uint8_t* bytes, std::size_t size) noexcept;

}  // namespace tianji_mapped_palm
