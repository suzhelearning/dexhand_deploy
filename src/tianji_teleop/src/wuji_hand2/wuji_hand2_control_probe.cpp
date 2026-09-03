#include <array>
#include <stdexcept>
#include <cstdint>
#include <iomanip>
#include <sstream>

#define private public
#include "wuji_hand2_control.cpp"
#undef private

namespace tianji_teleop
{
namespace
{
void require(bool condition, const char * message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

constexpr std::array<uint8_t, 20> kSparseNids{
  1, 2, 3, 4, 6, 7, 8, 9, 11, 12, 13, 14, 16, 17, 18, 19, 21, 22, 23, 24};

void valid_states_are_mapped_to_dense_finger_major()
{
  WujiHand2Device device;
  std::array<WujiJointStateEntry, 20> entries{};
  for (std::size_t wire = 0; wire < entries.size(); ++wire) {
    const std::size_t dense = (wire * 7) % entries.size();
    entries[wire].nid = kSparseNids[dense];
    entries[wire].position = static_cast<float>(dense + 1);
  }
  WujiJointStateFrame frame{};
  frame.joints = entries.data();
  frame.joints_len = entries.size();
  WujiHand2Device::on_joint_states(WUJI_FRAME_KIND_OK, &frame, &device);

  float position[20]{};
  float velocity[20]{};
  float effort[20]{};
  require(device.latest_states(position, velocity, effort), "probe condition failed");
  for (std::size_t dense = 0; dense < 20; ++dense) {
    require(position[dense] == static_cast<float>(dense + 1), "probe condition failed");
  }
}

void incomplete_and_duplicate_states_are_rejected()
{
  WujiHand2Device device;
  std::array<WujiJointStateEntry, 20> entries{};
  for (std::size_t dense = 0; dense < entries.size(); ++dense) {
    entries[dense].nid = kSparseNids[dense];
    entries[dense].position = 7.0F;
  }
  WujiJointStateFrame frame{};
  frame.joints = entries.data();
  frame.joints_len = entries.size();
  WujiHand2Device::on_joint_states(WUJI_FRAME_KIND_OK, &frame, &device);

  entries[19].nid = 21;  // 重复；缺少小指最后一个 motor。
  entries[19].position = 99.0F;
  WujiHand2Device::on_joint_states(WUJI_FRAME_KIND_OK, &frame, &device);
  float position[20]{};
  float velocity[20]{};
  float effort[20]{};
  uint64_t serial = 0;
  require(device.latest_states(position, velocity, effort, nullptr, &serial), "probe condition failed");
  require(serial == 1, "probe condition failed");
  require(position[19] == 7.0F, "probe condition failed");

  frame.joints_len = 19;
  WujiHand2Device::on_joint_states(WUJI_FRAME_KIND_OK, &frame, &device);
  require(device.latest_states(position, velocity, effort, nullptr, &serial), "probe condition failed");
  require(serial == 1, "probe condition failed");

  frame.joints_len = entries.size();
  entries[0].nid = 0;  // 旧的 0-based motor NID 非法。
  WujiHand2Device::on_joint_states(WUJI_FRAME_KIND_OK, &frame, &device);
  require(device.latest_states(position, velocity, effort, nullptr, &serial), "probe condition failed");
  require(serial == 1, "probe condition failed");

  entries[0].nid = 5;  // tactile 槽，不是 motor NID。
  WujiHand2Device::on_joint_states(WUJI_FRAME_KIND_OK, &frame, &device);
  require(device.latest_states(position, velocity, effort, nullptr, &serial), "probe condition failed");
  require(serial == 1, "probe condition failed");
}

void diagnostics_use_dense_mask_and_require_all_motors()
{
  EnableCtx ctx;
  std::array<WujiJointDiagnosticsEntry, 20> entries{};
  for (std::size_t dense = 0; dense < entries.size(); ++dense) {
    entries[dense].nid = kSparseNids[dense];
    entries[dense].vbus_v_fb = 0.0F;
    entries[dense].status_word = 0x102U;
    entries[dense].error_code_current = 0x1203U;
  }
  WujiJointDiagnosticsFrame frame{};
  frame.joints = entries.data();
  frame.joints_len = entries.size();

  entries[19].status_word = 0x101U;  // Ready, not Enabled.
  WujiHand2Device::on_diagnostics(WUJI_FRAME_KIND_OK, &frame, &ctx);
  require(ctx.online.load() == 0xFFFFFU, "probe condition failed");
  require(!ctx.enabled.load(), "probe condition failed");
  require(ctx.latest_diagnostics_reason.find("20/20 Enabled") != std::string::npos, "probe condition failed");

  entries[19].status_word = 0x102U;
  WujiHand2Device::on_diagnostics(WUJI_FRAME_KIND_OK, &frame, &ctx);
  require(ctx.online.load() == 0xFFFFFU, "probe condition failed");
  require(ctx.enabled.load(), "probe condition failed");
  require(ctx.latest_diagnostics_summary.find("nid=1") != std::string::npos, "probe condition failed");
  require(ctx.latest_diagnostics_summary.find("status_word=0x102") != std::string::npos, "probe condition failed");
  require(ctx.latest_diagnostics_summary.find("ext_state=2") != std::string::npos, "probe condition failed");
  require(ctx.latest_diagnostics_summary.find("status_word&3=2") != std::string::npos, "probe condition failed");
  require(ctx.latest_diagnostics_summary.find("vbus_v_fb=0") != std::string::npos, "probe condition failed");
  require(ctx.latest_diagnostics_summary.find("error_code_current=0x1203") !=
  std::string::npos, "probe condition failed");

  WujiHand2Device::on_diagnostics(WUJI_FRAME_KIND_END, nullptr, &ctx);
  require(ctx.online.load() == 0xFFFFFU, "probe condition failed");
  require(ctx.enabled.load(), "probe condition failed");  // Closing the subscription must not clear the latch.
  require(ctx.latest_diagnostics_summary.find("nid=1") != std::string::npos, "probe condition failed");
  require(ctx.latest_diagnostics_summary.find("nid=24") != std::string::npos, "probe condition failed");

  entries[19].status_word = 0x101U;
  frame.joints = entries.data();
  frame.joints_len = entries.size();
  WujiHand2Device::on_diagnostics(WUJI_FRAME_KIND_OK, &frame, &ctx);
  require(ctx.online.load() == 0xFFFFFU, "probe condition failed");
  require(ctx.enabled.load(), "probe condition failed");  // A later non-enabled frame must not clear it.
  entries[19].status_word = 0x102U;
  entries[19].nid = 21;  // 后续非法帧不能清除成功 latch。
  WujiHand2Device::on_diagnostics(WUJI_FRAME_KIND_OK, &frame, &ctx);
  require(ctx.online.load() == 0xFFFFFU, "probe condition failed");
  require(ctx.latest_diagnostics_summary.find("nid=21") != std::string::npos, "probe condition failed");
  require(ctx.enabled.load(), "probe condition failed");
  entries[19].nid = kSparseNids[19];

  EnableCtx invalid_ctx;
  entries[19].status_word = 0x102U;
  entries[19].nid = 21;  // 重复的小指 NID。
  WujiHand2Device::on_diagnostics(WUJI_FRAME_KIND_OK, &frame, &invalid_ctx);
  require(invalid_ctx.online.load() == 0, "probe condition failed");
  require(!invalid_ctx.enabled.load(), "probe condition failed");
  require(invalid_ctx.latest_diagnostics_reason.find("非法") != std::string::npos, "probe condition failed");
  const std::string timeout_detail = enable_timeout_detail(invalid_ctx);
  require(timeout_detail.find("nid=1") != std::string::npos, "probe condition failed");
  require(timeout_detail.find("status_word=0x102") != std::string::npos, "probe condition failed");
  require(timeout_detail.find("ext_state=2") != std::string::npos, "probe condition failed");
  require(timeout_detail.find("vbus_v_fb=0") != std::string::npos, "probe condition failed");
  require(timeout_detail.find("error_code_current=0x1203") != std::string::npos, "probe condition failed");

  entries[19].nid = kSparseNids[19];
  frame.joints_len = 19;  // missing one entry.
  WujiHand2Device::on_diagnostics(WUJI_FRAME_KIND_OK, &frame, &invalid_ctx);
  require(invalid_ctx.online.load() == 0, "probe condition failed");
  require(!invalid_ctx.enabled.load(), "probe condition failed");
  require(invalid_ctx.latest_diagnostics_reason.find("非法") != std::string::npos, "probe condition failed");

  frame.joints = nullptr;
  frame.joints_len = entries.size();
  WujiHand2Device::on_diagnostics(WUJI_FRAME_KIND_OK, &frame, &invalid_ctx);
  require(invalid_ctx.online.load() == 0, "probe condition failed");
  require(!invalid_ctx.enabled.load(), "probe condition failed");
  require(invalid_ctx.latest_diagnostics_summary.find("joints=null") != std::string::npos, "probe condition failed");
  require(enable_timeout_detail(invalid_ctx).find("非法") != std::string::npos, "probe condition failed");

  EnableCtx no_frame;
  WujiHand2Device::on_diagnostics(WUJI_FRAME_KIND_LAG, nullptr, &no_frame);
  require(no_frame.online.load() == 0, "probe condition failed");
  require(!no_frame.enabled.load(), "probe condition failed");
  const std::string no_frame_detail = enable_timeout_detail(no_frame);
  require(no_frame_detail.find("未收到 OK frame") != std::string::npos, "probe condition failed");
  require(no_frame.latest_diagnostics_reason.find("kind=1") != std::string::npos, "probe condition failed");
}
}  // namespace
}  // namespace tianji_teleop

int main()
{
  tianji_teleop::valid_states_are_mapped_to_dense_finger_major();
  tianji_teleop::incomplete_and_duplicate_states_are_rejected();
  tianji_teleop::diagnostics_use_dense_mask_and_require_all_motors();
  return 0;
}
