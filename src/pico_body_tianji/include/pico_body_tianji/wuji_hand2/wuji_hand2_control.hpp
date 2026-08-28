#pragma once
/* Wuji Hand 2 的 wuji-sdk C API 封装。
 *
 * 分两层：
 *  - WujiRetargeter：纯计算（21×3 MediaPipe 键点 → 20 关节角），
 *    不连接硬件，dry-run 与真机共用；
 *  - WujiHand2Device：扫描/连接/配置/使能/发布/订阅关节状态。
 *
 * 关节序约定（来自 wuji-sdk C 文档）：
 *  20 关节按“拇指 / 食指 / 中指 / 无名指 / 小指”finger-major，
 *  每指 [1, 2, 3, 4]（thumb: cmc_flex, cmc_abd, mcp, ip；
 *  指:  mcp_flex, mcp_abd, pip, dip），与 hand2_beta1 URDF 的
 *  revolute 关节声明顺序一致，即 firmware 序 == 组合 URDF 关节序。
 */
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>

#include <wuji_sdk.h>

namespace pico_body_tianji
{

inline constexpr std::size_t kWujiJointCount = 20;
inline constexpr std::size_t kWujiKeypointCount = 21; /* ×3 坐标 */
inline constexpr int32_t kWujiHandModel2 = WUJI_HAND_MODEL_WUJI_HAND2;

/** firmware 序 20 关节名（与 hand2_beta1 URDF 一致；仅诊断输出用）。 */
const char * wuji_hand2_joint_name(std::size_t index);

/** 21×3 MediaPipe 键点（米制）→ 20 关节角（rad，firmware 序）。
 *
 * step() 前先做腕部相对化（kp -= kp[0]，幂等）与可选旋转
 * （set_rotation_deg，语义同 wuji-retargeting 的 mediapipe_rotation：
 * 外部 XYZ 欧拉角，点按 p' = R p 旋转）。会话使用 SDK 内建
 * WUJI_HAND_MODEL_WUJI_HAND2 调参配置。
 */
class WujiRetargeter
{
public:
  WujiRetargeter(int32_t side_handedness, std::string * error);
  ~WujiRetargeter();

  WujiRetargeter(const WujiRetargeter &) = delete;
  WujiRetargeter & operator=(const WujiRetargeter &) = delete;

  /** 外部 XYZ 欧拉角（度）；默认 0。 */
  void set_rotation_deg(double x_deg, double y_deg, double z_deg);

  /** 键点（≥63 个 float，任一参考系）→ qpos[20]。失败返回 false。 */
  bool step(const float * keypoints, float * qpos_out, std::string * error);

  /** 跟踪丢失后清 warm-start 与滤波状态。 */
  void reset();

private:
  WujiRetargetSession * session_ = nullptr;
  float rotation_[9]{1.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 1.0F};
};

/** Wuji Hand 2 真机会话（连接 → 配置 MIT → 使能 → 发布命令 → 状态流）。 */
class WujiHand2Device
{
public:
  struct Options
  {
    std::string serial;    /* 按 SN 匹配；优先于 address */
    std::string address;   /* 直接 connect(host:port) */
    float kp = 3.0F;
    float kd = 0.05F;
    float effort_limit_amps = 1.5F;
    float enable_timeout_s = 5.0F;
  };

  WujiHand2Device() = default;
  ~WujiHand2Device();

  WujiHand2Device(const WujiHand2Device &) = delete;
  WujiHand2Device & operator=(const WujiHand2Device &) = delete;

  /** 扫描并连接（首次调用前必须 wuji_init）。失败 false + error。 */
  bool connect_device(const Options & options, std::string * error);

  /** 配置 effort limit + MIT kp/kd → enable → 等待全部在线关节 Enabled。 */
  bool enable_and_wait(std::string * error);

  /** 打开命令发布通道（使能后调用）。 */
  bool open_publisher(std::string * error);

  /** 发送 20 关节位姿命令（position/velocity=0/effort=0）。 */
  bool send(const float * qpos20, std::string * error);

  /** 取最近一次 joint_states 及其本机接收时间/递增帧序。 */
  bool latest_states(
    float position[20], float velocity[20], float effort[20],
    std::int64_t * received_ns = nullptr,
    std::uint64_t * serial = nullptr) const;

  /** 最近 diagnostics 的在线关节数与 bitmap（位 i = 关节 i 在线）。 */
  uint8_t online_joint_count() const;
  uint32_t online_mask() const;

  /** 依次：关命令通道 → disable（best-effort）→ 断开 → release。 */
  void close();

private:
  static void on_diagnostics(
    WujiFrameKind kind, const WujiJointDiagnosticsFrame * frame, void * user);
  static void on_joint_states(
    WujiFrameKind kind, const WujiJointStateFrame * frame, void * user);

  struct WujiDevice * dev_ = nullptr;
  struct WujiJointCommandPublisher * publisher_ = nullptr;
  struct WujiSub * diagnostics_sub_ = nullptr;
  struct WujiSub * states_sub_ = nullptr;
  Options options_{};
  mutable std::mutex mu_;
  float last_position_[20]{0.0F};
  float last_velocity_[20]{0.0F};
  float last_effort_[20]{0.0F};
  bool have_states_ = false;
  std::int64_t states_received_ns_ = 0;
  std::uint64_t states_serial_ = 0;
  uint8_t online_count_ = 0;
  uint32_t online_mask_ = 0;
};

}  // namespace pico_body_tianji
