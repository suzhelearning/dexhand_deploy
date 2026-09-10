// Offline oracle adapter. Link against the pinned ORIGINAL protocol + so3
// sources, not the migrated implementation. No network or device interfaces.
#include "tianji_qp_ik/pico_teleop_protocol.hpp"

#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

template <typename Derived>
void values(const Eigen::MatrixBase<Derived>& matrix) {
  for (Eigen::Index row = 0; row < matrix.rows(); ++row) {
    for (Eigen::Index col = 0; col < matrix.cols(); ++col) {
      std::cout << ' ' << matrix(row, col);
    }
  }
}

int main() {
  std::cout << std::setprecision(17);
  tianji_qp_ik::PicoTeleopStreamGate gate(0.15, 0.60);
  std::string hex;
  while (std::getline(std::cin, hex)) {
    if (hex.size() % 2 != 0) return 2;
    std::vector<std::uint8_t> packet;
    for (std::size_t i = 0; i < hex.size(); i += 2) {
      packet.push_back(static_cast<std::uint8_t>(std::stoul(hex.substr(i, 2), nullptr, 16)));
    }
    const auto result = tianji_qp_ik::decodePicoTeleopPacket(packet.data(), packet.size());
    if (!result.frame) {
      std::cout << "rejected" << std::endl;
      continue;
    }
    const auto& f = *result.frame;
    std::cout << "accepted " << f.sequence << ' ' << f.tracking_epoch
              << ' ' << f.source_timestamp_ns << ' ' << f.bridge_send_monotonic_ns
              << ' ' << f.user_button_pressed << ' ' << f.left_arm_direction.valid
              << ' ' << f.right_arm_direction.valid << ' ' << f.upper_limb_skeleton.valid
              << ' ' << f.upper_limb_skeleton.rotations_valid;
    values(f.left.position);
    values(f.left.rotation);
    values(f.right.position);
    values(f.right.rotation);
    values(f.left_arm_direction.direction);
    values(f.right_arm_direction.direction);
    for (const auto& point : f.upper_limb_skeleton.points) values(point);
    for (const auto& rotation : f.upper_limb_skeleton.rotations) values(rotation.toRotationMatrix());
    const auto decision = gate.evaluate(f);
    std::cout << ' ' << decision.accepted << ' ' << decision.epoch_changed
              << ' ' << static_cast<int>(decision.reason)
              << ' ' << decision.stream_discontinuity;
    std::cout << std::endl;
  }
  return 0;
}
