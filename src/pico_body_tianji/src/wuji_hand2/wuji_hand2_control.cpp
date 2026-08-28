/* WujiRetargeter / WujiHand2Device 实现（wuji-sdk C API）。 */
#include "pico_body_tianji/wuji_hand2/wuji_hand2_control.hpp"

#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>

namespace pico_body_tianji
{

namespace
{

constexpr float kPi = 3.14159265358979323846F;

/* OD 0x6001 ext_state (status_word & 0x3): 0=Init 1=Ready 2=Enabled 3=Stopped。 */
constexpr uint32_t kExtStateEnabled = 2U;
constexpr uint32_t kExtStateMask = 0x3U;

bool status_ok(WujiStatus status)
{
  return status == WUJI_STATUS_OK;
}

std::string last_error(const char * context)
{
  return std::string(context) + ": " + wuji_last_error();
}

uint64_t now_us()
{
  return static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now().time_since_epoch())
      .count());
}

/* 外部 XYZ 欧拉角（度）→ R，按“p' = R p”作用于点（列主序存储）。 */
std::array<float, 9> euler_xyz_degrees(double x_deg, double y_deg, double z_deg)
{
  const double sx = std::sin(x_deg * kPi / 180.0), cx = std::cos(x_deg * kPi / 180.0);
  const double sy = std::sin(y_deg * kPi / 180.0), cy = std::cos(y_deg * kPi / 180.0);
  const double sz = std::sin(z_deg * kPi / 180.0), cz = std::cos(z_deg * kPi / 180.0);
  std::array<float, 9> r;
  /* R = Rz(yaw) @ Ry(pitch) @ Rx(roll)，列主序 r[col*3+row]。 */
  r[0] = static_cast<float>(cz * cy);
  r[1] = static_cast<float>(sz * cy);
  r[2] = static_cast<float>(-sy);
  r[3] = static_cast<float>(cz * sy * sx - sz * cx);
  r[4] = static_cast<float>(sz * sy * sx + cz * cx);
  r[5] = static_cast<float>(cy * sx);
  r[6] = static_cast<float>(cz * sy * cx + sz * sx);
  r[7] = static_cast<float>(sz * sy * cx - cz * sx);
  r[8] = static_cast<float>(cy * cx);
  return r;
}

/** enable 等待上下文（诊断回调在主循环等待时被 SDK 工作线程调用）。 */
struct EnableCtx
{
  std::atomic<bool> enabled{false};
  std::atomic<uint32_t> online{0};
};

}  // namespace

const char * wuji_hand2_joint_name(std::size_t index)
{
  static const char * const kNames[20] = {
    "r_thumb_cmc_flex", "r_thumb_cmc_abd", "r_thumb_mcp", "r_thumb_ip",
    "r_index_finger_mcp_flex", "r_index_finger_mcp_abd",
    "r_index_finger_pip", "r_index_finger_dip",
    "r_middle_finger_mcp_flex", "r_middle_finger_mcp_abd",
    "r_middle_finger_pip", "r_middle_finger_dip",
    "r_ring_finger_mcp_flex", "r_ring_finger_mcp_abd",
    "r_ring_finger_pip", "r_ring_finger_dip",
    /* tianji_wuji2.urdf 命名；旧 hand2_beta1 资产为 r_pinky_finger_*。 */
    "r_pinky_mcp_flex", "r_pinky_mcp_abd",
    "r_pinky_pip", "r_pinky_dip",
  };
  return index < 20 ? kNames[index] : "?";
}

// ------------------------------------------------------------------ retarget

WujiRetargeter::WujiRetargeter(int32_t side_handedness, std::string * error)
{
  WujiRetargetSession * session = nullptr;
  const WujiStatus status =
    wuji_retarget_session_create(kWujiHandModel2, side_handedness, &session);
  if (!status_ok(status)) {
    if (error != nullptr) {
      *error = last_error("wuji_retarget_session_create");
    }
    return;
  }
  session_ = session;
}

WujiRetargeter::~WujiRetargeter()
{
  wuji_retarget_session_free(session_);
}

void WujiRetargeter::set_rotation_deg(double x_deg, double y_deg, double z_deg)
{
  const std::array<float, 9> r = euler_xyz_degrees(x_deg, y_deg, z_deg);
  std::copy(r.begin(), r.end(), rotation_);
}

bool WujiRetargeter::step(
  const float * keypoints, float * qpos_out, std::string * error)
{
  if (session_ == nullptr) {
    if (error != nullptr) {
      *error = "retarget session 未创建";
    }
    return false;
  }

  /* 腕部相对化（幂等）：wuji 文档建议以 wrist(0) 为原点。 */
  float normalized[63];
  const float * src = keypoints;
  if (std::isfinite(keypoints[0]) && std::isfinite(keypoints[1]) &&
      std::isfinite(keypoints[2])) {
    normalized[0] = 0.0F;
    normalized[1] = 0.0F;
    normalized[2] = 0.0F;
    for (std::size_t i = 3; i < 63; i += 3) {
      normalized[i] = keypoints[i] - keypoints[0];
      normalized[i + 1] = keypoints[i + 1] - keypoints[1];
      normalized[i + 2] = keypoints[i + 2] - keypoints[2];
    }
    src = normalized;
  }

  bool identity = true;
  for (std::size_t i = 0; i < 9; ++i) {
    const float expected = (i % 4 == 0) ? 1.0F : 0.0F;
    identity = identity && (std::abs(rotation_[i] - expected) < 1.0e-6F);
  }
  if (!identity) {
    float rotated[63];
    for (std::size_t p = 0; p < 21; ++p) {
      const float x = src[p * 3], y = src[p * 3 + 1], z = src[p * 3 + 2];
      rotated[p * 3] = rotation_[0] * x + rotation_[3] * y + rotation_[6] * z;
      rotated[p * 3 + 1] = rotation_[1] * x + rotation_[4] * y + rotation_[7] * z;
      rotated[p * 3 + 2] = rotation_[2] * x + rotation_[5] * y + rotation_[8] * z;
    }
    src = rotated;
  }

  const WujiStatus status = wuji_retarget_session_step(session_, src, qpos_out);
  if (!status_ok(status)) {
    if (error != nullptr) {
      *error = last_error("wuji_retarget_session_step");
    }
    return false;
  }
  return true;
}

void WujiRetargeter::reset()
{
  if (session_ != nullptr) {
    (void)wuji_retarget_session_reset(session_);
  }
}

// ------------------------------------------------------------------- device

WujiHand2Device::~WujiHand2Device()
{
  close();
}

bool WujiHand2Device::connect_device(
  const Options & options, std::string * error)
{
  options_ = options;
  WujiDiscovered * list = nullptr;
  size_t count = 0;
  if (!status_ok(wuji_scan(&list, &count))) {
    if (error != nullptr) {
      *error = last_error("wuji_scan");
    }
    return false;
  }

  WujiConnectTarget target{};
  bool have_target = false;
  if (!options.address.empty()) {
    target.kind = WUJI_CONNECT_TARGET_KIND_ADDR;
    target.value = options.address.c_str();
    have_target = true;
    printf("  --- Wuji Hand 2: 直接连接 %s\n", options.address.c_str());
  } else {
    size_t selected = count;
    for (size_t i = 0; i < count; ++i) {
      printf("  --- 扫描结果 %zu/%zu: SN=%s Type=%s Address=%s\n", i, count,
             list[i].serial_number, list[i].model, list[i].address);
      if (list[i].device_id == WUJI_DEVICE_TYPE_WUJI_HAND_2 &&
          (options.serial.empty() ||
           options.serial == std::string(list[i].serial_number))) {
        selected = i;
        break;
      }
    }
    if (selected == count) {
      wuji_discovered_free(list, count);
      if (error != nullptr) {
        *error = options.serial.empty()
                   ? "扫描未发现 Wuji Hand 2 设备"
                   : "未找到 SN 匹配的 Wuji Hand 2 设备（共 " +
                       std::to_string(count) + " 个扫描结果）";
      }
      return false;
    }
    target.kind = WUJI_CONNECT_TARGET_KIND_SN;
    target.value = list[selected].serial_number;
    have_target = true;
  }

  if (!have_target) {
    wuji_discovered_free(list, count);
    if (error != nullptr) {
      *error = "未指定连接地址或 SN，且扫描无结果";
    }
    return false;
  }

  const WujiStatus status =
    wuji_connect(&target, "wuji_hand_2", nullptr, &dev_);
  wuji_discovered_free(list, count);
  if (!status_ok(status)) {
    dev_ = nullptr;
    if (error != nullptr) {
      *error = last_error("wuji_connect");
    }
    return false;
  }
  return true;
}

bool WujiHand2Device::enable_and_wait(std::string * error)
{
  if (dev_ == nullptr) {
    if (error != nullptr) {
      *error = "设备未连接";
    }
    return false;
  }

  uint8_t online = 0;
  if (!status_ok(wuji_hand_2_online_joints_count(dev_, &online))) {
    if (error != nullptr) {
      *error = last_error("wuji_hand_2_online_joints_count");
    }
    return false;
  }
  if (online == 0) {
    if (error != nullptr) {
      *error = "Wuji Hand 2 在线关节为 0（检查供电/网络）";
    }
    return false;
  }
  {
    std::lock_guard<std::mutex> guard(mu_);
    online_count_ = online;
  }

  /* MIT 阻抗控制为固件默认模式；只需设 effort limit + kp/kd 后 enable。 */
  if (!status_ok(
        wuji_hand_2_set_all_effort_limit(dev_, options_.effort_limit_amps))) {
    if (error != nullptr) {
      *error = last_error("wuji_hand_2_set_all_effort_limit");
    }
    return false;
  }
  float kp[20], kd[20];
  std::fill(kp, kp + 20, options_.kp);
  std::fill(kd, kd + 20, options_.kd);
  if (!status_ok(wuji_hand_2_set_all_mit_params(dev_, kp, kd))) {
    if (error != nullptr) {
      *error = last_error("wuji_hand_2_set_all_mit_params");
    }
    return false;
  }
  if (!status_ok(wuji_hand_2_enable(dev_, nullptr))) {
    if (error != nullptr) {
      *error = last_error("wuji_hand_2_enable");
    }
    return false;
  }

  /* 订阅 joint_diagnostics，等待全部在线关节 ext_state=Enabled。
   * 过滤 vbus_v_fb > 0.5（存在连接但逆变器未上电的关节）。 */
  EnableCtx ctx;
  if (!status_ok(wuji_hand_2_subscribe_joint_diagnostics(
          dev_, &WujiHand2Device::on_diagnostics, &ctx, &diagnostics_sub_))) {
    if (error != nullptr) {
      *error = last_error("wuji_hand_2_subscribe_joint_diagnostics");
      (void)wuji_hand_2_disable(dev_, nullptr);
      return false;
    }
  }

  const uint64_t deadline =
    now_us() + static_cast<uint64_t>(options_.enable_timeout_s * 1.0e6);
  while (!ctx.enabled.load(std::memory_order_relaxed) && now_us() < deadline) {
    usleep(50000);
  }
  wuji_sub_close(diagnostics_sub_);
  diagnostics_sub_ = nullptr;
  if (!ctx.enabled.load(std::memory_order_relaxed)) {
    if (error != nullptr) {
      *error =
        "电机使能超时（在线关节未全部进入 Enabled，检查供电/急停/网线）";
    }
    (void)wuji_hand_2_disable(dev_, nullptr);
    return false;
  }
  {
    std::lock_guard<std::mutex> guard(mu_);
    online_mask_ = ctx.online.load(std::memory_order_relaxed);
  }
  return true;
}

bool WujiHand2Device::open_publisher(std::string * error)
{
  if (dev_ == nullptr) {
    if (error != nullptr) {
      *error = "设备未连接";
    }
    return false;
  }
  if (!status_ok(wuji_hand_2_joint_command_publish(dev_, &publisher_))) {
    if (error != nullptr) {
      *error = last_error("wuji_hand_2_joint_command_publish");
    }
    return false;
  }
  /* 命令通道打开后再订阅状态流（与 SDK 示例一致：先通信后订阅）。 */
  if (!status_ok(wuji_hand_2_subscribe_joint_states(
          dev_, &WujiHand2Device::on_joint_states, this, &states_sub_))) {
    if (error != nullptr) {
      *error = last_error("wuji_hand_2_subscribe_joint_states");
    }
    wuji_joint_command_publisher_close(publisher_);
    publisher_ = nullptr;
    return false;
  }
  return true;
}

bool WujiHand2Device::send(const float * qpos20, std::string * error)
{
  if (publisher_ == nullptr) {
    if (error != nullptr) {
      *error = "命令通道未打开";
    }
    return false;
  }
  WujiJointCommand cmds[20];
  for (std::size_t i = 0; i < 20; ++i) {
    cmds[i].position = qpos20[i];
    cmds[i].velocity = 0.0F;
    cmds[i].effort = 0.0F;
  }
  if (!status_ok(wuji_joint_command_publisher_send(publisher_, cmds))) {
    if (error != nullptr) {
      *error = last_error("wuji_joint_command_publisher_send");
    }
    return false;
  }
  return true;
}

bool WujiHand2Device::latest_states(
  float position[20], float velocity[20], float effort[20]) const
{
  std::lock_guard<std::mutex> guard(mu_);
  if (!have_states_) {
    return false;
  }
  std::memcpy(position, last_position_, sizeof(last_position_));
  std::memcpy(velocity, last_velocity_, sizeof(last_velocity_));
  std::memcpy(effort, last_effort_, sizeof(last_effort_));
  return true;
}

uint8_t WujiHand2Device::online_joint_count() const
{
  std::lock_guard<std::mutex> guard(mu_);
  return online_count_;
}

uint32_t WujiHand2Device::online_mask() const
{
  std::lock_guard<std::mutex> guard(mu_);
  return online_mask_;
}

void WujiHand2Device::close()
{
  if (states_sub_ != nullptr) {
    wuji_sub_close(states_sub_);
    states_sub_ = nullptr;
  }
  if (publisher_ != nullptr) {
    wuji_joint_command_publisher_close(publisher_);
    publisher_ = nullptr;
  }
  if (dev_ != nullptr) {
    /* 断电前松开电机。 */
    (void)wuji_hand_2_disable(dev_, nullptr);
    (void)wuji_dev_disconnect(dev_);
    wuji_dev_release(dev_);
    dev_ = nullptr;
  }
}

void WujiHand2Device::on_diagnostics(
  WujiFrameKind kind, const WujiJointDiagnosticsFrame * frame, void * user)
{
  if (kind != WUJI_FRAME_KIND_OK || frame == nullptr || frame->joints == nullptr) {
    return;
  }
  if (frame->joints_len == 0) {
    return;
  }
  uint32_t online = 0;
  bool all_enabled = true;
  for (size_t i = 0; i < frame->joints_len; ++i) {
    const WujiJointDiagnosticsEntry & entry = frame->joints[i];
    if (entry.vbus_v_fb > 0.5F) {
      if (entry.nid < 20) {
        online |= (1U << entry.nid);
      }
      if ((entry.status_word & kExtStateMask) != kExtStateEnabled) {
        all_enabled = false;
      }
    }
  }
  if (online == 0) {
    return;
  }
  EnableCtx * ctx = static_cast<EnableCtx *>(user);
  ctx->online.store(online, std::memory_order_relaxed);
  if (all_enabled) {
    ctx->enabled.store(true, std::memory_order_relaxed);
  }
}

void WujiHand2Device::on_joint_states(
  WujiFrameKind kind, const WujiJointStateFrame * frame, void * user)
{
  if (kind != WUJI_FRAME_KIND_OK || frame == nullptr || frame->joints == nullptr) {
    return;
  }
  WujiHand2Device * self = static_cast<WujiHand2Device *>(user);
  float position[20];
  float velocity[20];
  float effort[20];
  std::fill(position, position + 20, 0.0F);
  std::fill(velocity, velocity + 20, 0.0F);
  std::fill(effort, effort + 20, 0.0F);
  for (size_t i = 0; i < frame->joints_len; ++i) {
    const WujiJointStateEntry & entry = frame->joints[i];
    if (entry.nid >= 20) {
      continue;
    }
    position[entry.nid] = entry.position;
    velocity[entry.nid] = entry.velocity;
    effort[entry.nid] = entry.effort;
  }
  std::lock_guard<std::mutex> guard(self->mu_);
  std::memcpy(self->last_position_, position, sizeof(position));
  std::memcpy(self->last_velocity_, velocity, sizeof(velocity));
  std::memcpy(self->last_effort_, effort, sizeof(effort));
  self->have_states_ = true;
}

}  // namespace pico_body_tianji
