/* wuji_hand2_bridge：Manus 键点 → wuji-sdk retarget → Wuji Hand 2 控制。
 *
 * 数据流（Zenoh，全部无前导斜杠）：
 *   pico_body_sim/{side}_hand/keypoints       ← 主机（H5 回放/动捕）发布
 *       21×3 float32 LE（米，腕部相对，MediaPipe 序）
 *   pico_body_sim/{side}_hand/joint_commands → 本节点发布（retarget 输出，
 *       20×float32 rad，firmware 序；dry-run 与真机一致，供 MuJoCo/诊断用）
 *   pico_body_real/{side}_hand/joint_states  → 本节点发布（真机 joint_states）
 *   pico_body_real/{side}_hand/status        → 本节点发布（JSON 文本）
 *
 * 真机路径（默认）：扫描/连接 wuji2 → 设 effort limit + MIT kp/kd →
 * enable → 等全部在线关节 Enabled → 打开命令通道 → 循环
 * “取最新键点 → retarget → 发送”。无键点帧时保持上一次命令（首帧为零）。
 * --dry-run：跳过硬件，只做 retarget 并发布命令（仿真验收用）。
 *
 * 安全：Ctrl+C 先关闭命令通道再 disable（松开电机）；异常路径同样
 * 保证 disable/disconnect/release。
 */
#include "pico_body_tianji/wuji_hand2/wuji_hand2_control.hpp"

#include <zenoh.hxx>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace
{

using pico_body_tianji::kWujiJointCount;
using pico_body_tianji::kWujiKeypointCount;
using pico_body_tianji::WujiHand2Device;
using pico_body_tianji::WujiRetargeter;

constexpr std::size_t kKeypointBytes = kWujiKeypointCount * 3 * sizeof(float);
constexpr std::size_t kJointBytes = kWujiJointCount * sizeof(float);

volatile std::sig_atomic_t g_stop = 0;
void on_sigint(int)
{
  g_stop = 1;
}

std::string strip_leading_slash(const std::string & ros_topic)
{
  return !ros_topic.empty() && ros_topic.front() == '/'
           ? ros_topic.substr(1)
           : ros_topic;
}

class WujiHand2Bridge
{
public:
  struct Params
  {
    std::string side = "right";
    std::string serial;
    std::string address;
    bool dry_run = false;
    float kp = 3.0F;
    float kd = 0.05F;
    float effort_limit_amps = 1.5F;
    float enable_timeout_s = 5.0F;
    int rate_hz = 100;
    float hold_timeout_s = 1.0F;
    float keypoint_timeout_s = 0.5F;
    float command_slew_rate_rad_s = 1.0F;
    float tracking_slew_rate_rad_s = 6.0F;
    float teleop_grace_s = 0.3F;
    bool log_qpos = false;
    float rotation_deg[3]{0.0F, 0.0F, 0.0F};
    std::string keypoints_key = "pico_body_sim/right_hand/keypoints";
    std::string commands_key = "pico_body_sim/right_hand/joint_commands";
    std::string states_key = "pico_body_real/right_hand/joint_states";
    std::string status_key = "pico_body_real/right_hand/status";
    std::string teleop_state_key = "pico_body/teleop_state";
  };

  explicit WujiHand2Bridge(const Params & params) : params_(params) {}

  int run();

private:
  void on_keypoints(const zenoh::Sample & sample);
  void on_teleop_state(const zenoh::Sample & sample);
  void publish_commands(const float qpos[20]);
  void publish_status(
    const std::string & phase, bool enabled, double keypoint_age_s,
    float keypoint_hz, uint8_t online_count, uint32_t online_mask,
    const std::string & teleop_state, bool tracking_allowed,
    bool keypoint_timed_out, float command_max_abs);

  Params params_;
  zenoh::Session session_{
    zenoh::Session::open(zenoh::Config::create_default())};
  std::unique_ptr<zenoh::Publisher> commands_pub_;
  std::unique_ptr<zenoh::Publisher> states_pub_;
  std::unique_ptr<zenoh::Publisher> status_pub_;
  std::unique_ptr<zenoh::LivelinessToken> liveliness_token_;

  std::mutex kp_mu_;
  std::array<float, kKeypointBytes / sizeof(float)> latest_kp_{};
  std::atomic<bool> have_kp_{false};
  std::chrono::steady_clock::time_point kp_stamp_{};
  std::chrono::steady_clock::time_point last_kp_received_{};
  std::atomic<std::uint64_t> kp_frames_{0};
  std::mutex state_mu_;
  std::string teleop_state_{"unknown"};
};

int WujiHand2Bridge::run()
{
  /* SDK 全局初始化（retarget 会话与设备扫描共用）。 */
  const WujiInitOptions init_opts{.log_level = 3};
  if (wuji_init(&init_opts) != WUJI_STATUS_OK) {
    std::cerr << "wuji_init failed: " << wuji_last_error() << std::endl;
    return 1;
  }

  const bool right = params_.side != "left";
  const int32_t handedness =
    right ? WUJI_HANDEDNESS_RIGHT : WUJI_HANDEDNESS_LEFT;

  std::string error;
  WujiRetargeter retargeter(handedness, &error);
  if (!error.empty()) {
    std::cerr << "retarget session 创建失败: " << error << std::endl;
    wuji_shutdown();
    return 1;
  }
  retargeter.set_rotation_deg(
    params_.rotation_deg[0], params_.rotation_deg[1], params_.rotation_deg[2]);

  /* 真机路径。 */
  std::unique_ptr<WujiHand2Device> device;
  if (!params_.dry_run) {
    device = std::make_unique<WujiHand2Device>();
    WujiHand2Device::Options options;
    options.serial = params_.serial;
    options.address = params_.address;
    options.kp = params_.kp;
    options.kd = params_.kd;
    options.effort_limit_amps = params_.effort_limit_amps;
    options.enable_timeout_s = params_.enable_timeout_s;
    if (!device->connect_device(options, &error)) {
      std::cerr << "连接 Wuji Hand 2 失败: " << error << std::endl;
      device->close();
      wuji_shutdown();
      return 1;
    }
    if (!device->enable_and_wait(&error)) {
      std::cerr << "使能失败: " << error << std::endl;
      device->close();
      wuji_shutdown();
      return 1;
    }
    if (!device->open_publisher(&error)) {
      std::cerr << "打开命令通道失败: " << error << std::endl;
      device->close();
      wuji_shutdown();
      return 1;
    }
    printf(
      "  --- 已使能（在线 %u 关节，mask=0x%08x），命令通道已打开\n",
      device->online_joint_count(), device->online_mask());
  } else {
    printf("  --- dry-run：不连接硬件，仅 retarget 并发布命令。\n");
  }

  /* Zenoh 发布/订阅。 */
  commands_pub_ = std::make_unique<zenoh::Publisher>(
    session_.declare_publisher(
      zenoh::KeyExpr(strip_leading_slash(params_.commands_key))));
  states_pub_ = std::make_unique<zenoh::Publisher>(
    session_.declare_publisher(
      zenoh::KeyExpr(strip_leading_slash(params_.states_key))));
  status_pub_ = std::make_unique<zenoh::Publisher>(
    session_.declare_publisher(
      zenoh::KeyExpr(strip_leading_slash(params_.status_key))));
  liveliness_token_ = std::make_unique<zenoh::LivelinessToken>(
    session_.liveliness_declare_token(zenoh::KeyExpr("tj/live/wuji_hand2_bridge")));
  zenoh::Subscriber keypoints_sub = session_.declare_subscriber(
    zenoh::KeyExpr(strip_leading_slash(params_.keypoints_key)),
    [this](const zenoh::Sample & sample) { on_keypoints(sample); },
    []() {});
  zenoh::Subscriber teleop_state_sub = session_.declare_subscriber(
    zenoh::KeyExpr(strip_leading_slash(params_.teleop_state_key)),
    [this](const zenoh::Sample & sample) { on_teleop_state(sample); },
    []() {});

  publish_status(
    "zero_hold", !params_.dry_run, 1.0e9, 0.0F,
    device ? device->online_joint_count() : 0,
    device ? device->online_mask() : 0,
    "unknown", false, true, 0.0F);
  printf(
    "  --- 桥接运行中（side=%s rate=%dHz dry_run=%s）；订阅 %s\n",
    params_.side.c_str(), params_.rate_hz, params_.dry_run ? "是" : "否",
    params_.keypoints_key.c_str());
  printf(
    "  --- mediapipe_rotation=(%.1f, %.1f, %.1f) 度\n",
    params_.rotation_deg[0], params_.rotation_deg[1], params_.rotation_deg[2]);

  const double tick_duration = 1.0 / static_cast<double>(params_.rate_hz);
  const auto tick_ns =
    std::chrono::nanoseconds(static_cast<int64_t>(tick_duration * 1.0e9));
  auto next_tick = std::chrono::steady_clock::now();
  auto last_status = next_tick;

  float retarget_qpos[20] = {0.0F};
  float command_qpos[20] = {0.0F};
  bool have_last_qpos = false;
  std::string last_control_phase;
  std::uint64_t command_count = 0;
  const float max_command_step =
    params_.command_slew_rate_rad_s / static_cast<float>(params_.rate_hz);
  const float max_tracking_step =
    params_.tracking_slew_rate_rad_s / static_cast<float>(params_.rate_hz);
  auto last_tracking_at = std::chrono::steady_clock::now();

  while (!g_stop) {
    next_tick += tick_ns;
    const auto now = std::chrono::steady_clock::now();
    double kp_age_s = 1.0e9;
    {
      std::lock_guard<std::mutex> guard(kp_mu_);
      if (last_kp_received_ != std::chrono::steady_clock::time_point{}) {
        kp_age_s =
          std::chrono::duration<double>(now - last_kp_received_).count();
      }
    }

    /* 取最新键点（drain-latest），retarget 结果只缓存，不直接发送。 */
    {
      std::lock_guard<std::mutex> guard(kp_mu_);
      if (have_kp_.load(std::memory_order_relaxed)) {
        std::string step_error;
        const bool fresh = retargeter.step(
          latest_kp_.data(), retarget_qpos, &step_error);
        if (!fresh) {
          fprintf(stderr, "retarget step 失败: %s\n", step_error.c_str());
        } else {
          have_last_qpos = true;
          ++kp_frames_;
        }
        have_kp_.store(false, std::memory_order_relaxed);
      }
    }

    std::string teleop_state;
    {
      std::lock_guard<std::mutex> guard(state_mu_);
      teleop_state = teleop_state_;
    }
    const bool keypoint_timed_out =
      kp_age_s > static_cast<double>(params_.keypoint_timeout_s);
    const bool tracking_allowed =
      teleop_state == "teleop" && have_last_qpos && !keypoint_timed_out;
    if (tracking_allowed) {
      last_tracking_at = now;
    }
    const double teleop_elapsed_s =
      std::chrono::duration<double>(now - last_tracking_at).count();
    /* 短暂离开 teleop（grace 内）保持当前命令：上游状态抖动不再触发
     * 周期回零；超过 grace 才以回零速率缓速回零。 */
    const bool holding =
      !tracking_allowed && teleop_elapsed_s < params_.teleop_grace_s;

    /* teleop 跟踪 retarget；holding 保持；其余缓速回零。 */
    float command_max_abs = 0.0F;
    for (std::size_t i = 0; i < kWujiJointCount; ++i) {
      const float desired = tracking_allowed ? retarget_qpos[i] :
        (holding ? command_qpos[i] : 0.0F);
      const float step =
        tracking_allowed ? max_tracking_step : max_command_step;
      const float delta = std::clamp(
        desired - command_qpos[i], -step, step);
      command_qpos[i] += delta;
      command_max_abs = std::max(command_max_abs, std::abs(command_qpos[i]));
    }
    const std::string control_phase = tracking_allowed ? "tracking" :
      (holding ? "hold" :
       (command_max_abs > 1.0e-3F ? "returning_zero" : "zero_hold"));
    if (control_phase != last_control_phase) {
      printf(
        "  --- 手桥 phase=%s teleop_state=%s keypoint_age=%.3fs grace=%.3fs\n",
        control_phase.c_str(), teleop_state.c_str(), kp_age_s,
        teleop_elapsed_s);
      last_control_phase = control_phase;
    }

    publish_commands(command_qpos);
    if (device != nullptr) {
      std::string send_error;
      if (!device->send(command_qpos, &send_error)) {
        fprintf(stderr, "发送命令失败: %s\n", send_error.c_str());
        g_stop = 1;
      }
    }
    ++command_count;
    if (params_.log_qpos) {
      std::ostringstream stream;
      stream << std::fixed << std::setprecision(3) << "[";
      for (std::size_t i = 0; i < kWujiJointCount; ++i) {
        stream << (i == 0 ? "" : ",") << command_qpos[i];
      }
      stream << "]";
      printf("  --- qpos: %s\n", stream.str().c_str());
    }

    /* 真机状态回读。 */
    if (device != nullptr) {
      float position[20], velocity[20], effort[20];
      if (device->latest_states(position, velocity, effort)) {
        std::vector<std::uint8_t> payload(kJointBytes);
        std::memcpy(payload.data(), position, kJointBytes);
        states_pub_->put(std::move(payload));
      }
    }

    if (now >= last_status + std::chrono::milliseconds(500)) {
      const double window = 0.5;
      const float keypoint_hz =
        static_cast<float>(kp_frames_.exchange(0) / window);
      publish_status(
        control_phase, !params_.dry_run, kp_age_s,
        keypoint_hz, device ? device->online_joint_count() : 0,
        device ? device->online_mask() : 0,
        teleop_state, tracking_allowed, keypoint_timed_out,
        command_max_abs);
      last_status = now;
    }

    const auto target = std::max(next_tick, now);
    std::this_thread::sleep_until(target);
  }

  printf("  --- 收到停止信号，开始安全关闭...\n");
  if (device != nullptr) {
    device->close();
  }
  publish_status(
    "stopped", false, 0.0, 0.0F, 0, 0,
    "stopped", false, false, 0.0F);
  wuji_shutdown();
  printf(
    "  --- Wuji Hand 2 桥已退出（命令数=%llu）\n",
    static_cast<unsigned long long>(command_count));
  return 0;
}

void WujiHand2Bridge::on_keypoints(const zenoh::Sample & sample)
{
  const zenoh::Bytes & payload = sample.get_payload();
  if (payload.size() != kKeypointBytes) {
    return;
  }
  zenoh::Bytes::Reader reader = payload.reader();
  std::lock_guard<std::mutex> guard(kp_mu_);
  const size_t read = reader.read(
    reinterpret_cast<std::uint8_t *>(latest_kp_.data()), kKeypointBytes);
  if (read != kKeypointBytes) {
    return;
  }
  kp_stamp_ = std::chrono::steady_clock::now();
  last_kp_received_ = kp_stamp_;
  have_kp_.store(true, std::memory_order_relaxed);
}

void WujiHand2Bridge::on_teleop_state(const zenoh::Sample & sample)
{
  const std::string state = sample.get_payload().as_string();
  if (state.empty()) {
    return;
  }
  std::lock_guard<std::mutex> guard(state_mu_);
  teleop_state_ = state;
}

void WujiHand2Bridge::publish_commands(const float qpos[20])
{
  std::vector<std::uint8_t> payload(kJointBytes);
  std::memcpy(payload.data(), qpos, kJointBytes);
  commands_pub_->put(std::move(payload));
}

void WujiHand2Bridge::publish_status(
  const std::string & phase, bool enabled, double keypoint_age_s,
  float keypoint_hz, uint8_t online_count, uint32_t online_mask,
  const std::string & teleop_state, bool tracking_allowed,
  bool keypoint_timed_out, float command_max_abs)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(1);
  stream << "{\"phase\":\"" << phase
         << "\",\"enabled\":" << (enabled ? "true" : "false")
         << ",\"dry_run\":" << (params_.dry_run ? "true" : "false")
         << ",\"side\":\"" << params_.side << "\""
         << ",\"rate_hz\":" << params_.rate_hz
         << ",\"keypoints_age_s\":" << keypoint_age_s
         << ",\"keypoints_hz\":" << keypoint_hz
         << ",\"teleop_state\":\"" << teleop_state << "\""
         << ",\"tracking_allowed\":"
         << (tracking_allowed ? "true" : "false")
         << ",\"keypoint_timed_out\":"
         << (keypoint_timed_out ? "true" : "false")
         << ",\"command_max_abs_rad\":" << command_max_abs
         << ",\"joints_online\":" << static_cast<int>(online_count)
         << ",\"online_mask\":\"0x" << std::hex << std::setw(8)
         << std::setfill('0') << online_mask << std::dec << "\""
         << ",\"rotation_deg\":[" << params_.rotation_deg[0] << ","
         << params_.rotation_deg[1] << "," << params_.rotation_deg[2] << "]}";
  status_pub_->put(stream.str());
}

}  // namespace

int main(int argc, char ** argv)
{
  struct sigaction action;
  std::memset(&action, 0, sizeof(action));
  action.sa_handler = on_sigint;
  sigemptyset(&action.sa_mask);
  action.sa_flags = 0;
  if (sigaction(SIGINT, &action, nullptr) != 0) {
    std::cerr << "sigaction(SIGINT) failed" << std::endl;
    return 1;
  }

  WujiHand2Bridge::Params params;
  bool custom_keys = false;
  bool explicit_rotation = false;
  try {
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      auto require_value = [&]() -> std::string {
        if (i + 1 >= argc) {
          throw std::invalid_argument(arg + " 缺少值");
        }
        return argv[++i];
      };
      if (arg == "--side") {
        params.side = require_value();
      } else if (arg == "--serial") {
        params.serial = require_value();
      } else if (arg == "--address") {
        params.address = require_value();
      } else if (arg == "--kp") {
        params.kp = std::stof(require_value());
      } else if (arg == "--kd") {
        params.kd = std::stof(require_value());
      } else if (arg == "--effort-limit") {
        params.effort_limit_amps = std::stof(require_value());
      } else if (arg == "--enable-timeout") {
        params.enable_timeout_s = std::stof(require_value());
      } else if (arg == "--rate") {
        params.rate_hz = std::stoi(require_value());
      } else if (arg == "--hold-timeout") {
        params.hold_timeout_s = std::stof(require_value());
      } else if (arg == "--keypoint-timeout") {
        params.keypoint_timeout_s = std::stof(require_value());
      } else if (arg == "--command-slew-rate") {
        params.command_slew_rate_rad_s = std::stof(require_value());
      } else if (arg == "--tracking-slew-rate") {
        params.tracking_slew_rate_rad_s = std::stof(require_value());
      } else if (arg == "--teleop-grace-s") {
        params.teleop_grace_s = std::stof(require_value());
      } else if (arg == "--rotation-x") {
        params.rotation_deg[0] = std::stof(require_value());
        explicit_rotation = true;
      } else if (arg == "--rotation-y") {
        params.rotation_deg[1] = std::stof(require_value());
        explicit_rotation = true;
      } else if (arg == "--rotation-z") {
        params.rotation_deg[2] = std::stof(require_value());
        explicit_rotation = true;
      } else if (arg == "--keypoints-key") {
        params.keypoints_key = require_value();
        custom_keys = true;
      } else if (arg == "--commands-key") {
        params.commands_key = require_value();
        custom_keys = true;
      } else if (arg == "--states-key") {
        params.states_key = require_value();
        custom_keys = true;
      } else if (arg == "--status-key") {
        params.status_key = require_value();
        custom_keys = true;
      } else if (arg == "--teleop-state-key") {
        params.teleop_state_key = require_value();
        custom_keys = true;
      } else if (arg == "--dry-run") {
        params.dry_run = true;
      } else if (arg == "--log-qpos") {
        params.log_qpos = true;
      } else if (arg == "-h" || arg == "--help") {
        printf(
          "用法: wuji_hand2_bridge [选项]\n"
          "  --side right|left          手侧（默认 right）\n"
          "  --kp / --kd / --effort-limit  MIT 增益与电流上限\n"
          "  --rate N                    命令频率（默认 100 Hz）\n"
          "  --serial SN / --address HOST:PORT  设备选择\n"
          "  --dry-run                   不连接硬件（仿真/测试）\n"
          "  --rotation-x/y/z DEG        mediapipe_rotation\n"
          "  --keypoints-key / --commands-key / --states-key / --status-key\n"
          "  --keypoint-timeout S        teleop 键点超时后回零（默认 0.5s）\n"
          "  --command-slew-rate RAD_S   回零最大速度（默认 1rad/s）\n"
          "  --tracking-slew-rate RAD_S  跟踪最大速度（默认 6rad/s）\n"
          "  --teleop-grace-s S          离开 teleop 后保持命令窗口（默认 0.3s）\n"
          "  --teleop-state-key KEY      idle/teleop/returning 状态话题\n");
        return 0;
      } else {
        std::cerr << "未知参数: " << arg << std::endl;
        return 2;
      }
    }
  } catch (const std::exception & exception) {
    std::cerr << "参数错误: " << exception.what() << std::endl;
    return 2;
  }

  if (params.side != "left" && params.side != "right") {
    std::cerr << "--side 必须为 left 或 right" << std::endl;
    return 2;
  }
  if (params.rate_hz < 1 || params.rate_hz > 500) {
    std::cerr << "--rate 超出范围（1..500）" << std::endl;
    return 2;
  }
  if (!(params.keypoint_timeout_s > 0.0F) ||
      !(params.command_slew_rate_rad_s > 0.0F) ||
      !(params.tracking_slew_rate_rad_s > 0.0F) ||
      !(params.teleop_grace_s >= 0.0F)) {
    std::cerr << "--keypoint-timeout / --command-slew-rate / "
                 "--tracking-slew-rate 必须为正数，"
                 "--teleop-grace-s 必须非负"
              << std::endl;
    return 2;
  }
  /* 未自定义话题时按 side 派生默认值。 */
  if (!custom_keys) {
    params.keypoints_key = "pico_body_sim/" + params.side + "_hand/keypoints";
    params.commands_key = "pico_body_sim/" + params.side + "_hand/joint_commands";
    params.states_key = "pico_body_real/" + params.side + "_hand/joint_states";
    params.status_key = "pico_body_real/" + params.side + "_hand/status";
  }
  /* Manus 输入按 wuji-retargeting retarget_manus_{left,right}.yaml 的
   * mediapipe_rotation 默认：右 z=-15°、左 z=+15°；--rotation-* 可覆盖。 */
  if (!explicit_rotation) {
    params.rotation_deg[2] = params.side == "right" ? -15.0F : 15.0F;
  }

  try {
    WujiHand2Bridge bridge(params);
    return bridge.run();
  } catch (const std::exception & exception) {
    std::cerr << "wuji_hand2_bridge 失败: " << exception.what() << std::endl;
    return 1;
  }
}
