#pragma once
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include "arm_motor_bus.h"

namespace r1_arm_hardware
{

constexpr int kArmJointCount = 7;

/**
 * R1-7a 机械臂 ros2_control 硬件插件(SystemInterface)。
 *
 * 总线(ArmMotorBus)内部已完成标定,所有 q/dq/tau 均为 URDF 坐标系,
 * 插件不再做任何 dir/scale 换算。
 *
 * 状态机(在 write() 内异步推进,on_activate 只置位不阻塞):
 *   IDLE → PROBE(零力矩 FOC 探测反馈 + 捕获位姿合理性校验,越界/NaN 拒绝使能)
 *        → ENABLE(FOC 保持,增益渐进爬升 + 目标步进限幅 + 漂移保护,防猛拉)
 *        → READY(转发 joint_trajectory_controller 指令,带四重保护)
 *   任一相位超时/故障 → FAULT(持续发 mode=0 停止包,等待清障或人工干预)
 *
 * 串口独占防护:打开前扫 /proc 检测其他进程占用(拒绝打开),打开后持守卫 fd
 * 设 TIOCEXCL(其他进程无法再打开本端口)——杜绝双程序争抢串口的事故场景。
 */
class R1ArmHardware : public hardware_interface::SystemInterface
{
public:
  R1ArmHardware();
  ~R1ArmHardware() override;

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  enum class ArmPhase { IDLE, PROBE, ENABLE, READY, FAULT };

  // 发包辅助(与 SDK test_arm_lowlevel 的 sendStop/sendZeroTorque/sendFocPosition 一致)
  void sendStopCmds();
  void sendZeroTorqueCmds();
  // q 为总线系(模板标定后、补偿前)目标位姿;kp/kd 为本次发包用的增益
  void sendFocHold(const std::array<float, kArmJointCount> & q, float kp, float kd);
  void enterFault(const std::string & reason);

  // ---- 串口独占防护(2026-10-28 撞限位事故的直接教训:两个程序争用同一串口,
  //      数据打架 → PROBE 读到脏位姿 → ENABLE 按错误目标猛拉) ----
  // 扫描 /proc 找当前正打开本端口的其他进程(按设备号匹配,兼容符号链接别名);
  // 返回占用者 pid,未占用返回 -1,端口设备文件不存在返回 0
  pid_t findPortOwner() const;
  std::string readProcComm(pid_t pid) const;
  // 总线打开成功后调用:开守卫 fd 并设 TIOCEXCL,此后其他进程 open() 本端口一律 EBUSY
  bool lockPortExclusive();
  void unlockPortExclusive();

  // 参数读取(带默认值回退)
  std::string getParamStr(const std::string & name, const std::string & def) const;
  double getParamDouble(const std::string & name, double def) const;
  int getParamInt(const std::string & name, int def) const;

  hardware_interface::HardwareInfo info_;
  std::vector<double> hw_joint_commands_;
  std::vector<double> hw_joint_states_;
  std::vector<double> hw_joint_states_vel_;

  std::unique_ptr<ArmMotorBus> motor_bus_;
  std::array<ArmJointCmd, kArmJointCount> cached_cmds_;

  // ---- 参数 ----
  std::string port_ = "/dev/ttyUSB0";
  int baud_ = 6000000;
  std::string calib_path_;
  double kp_ = 60.0;
  double kd_ = 0.5;
  double max_lead_ = 0.15;   // 每 tick 命令超前上限 (rad)
  int probe_timeout_ms_ = 3000;
  int enable_timeout_ms_ = 1000;   // 增益爬升完成后的附加宽限时长
  int stop_stream_ms_ = 500; // 停机前 mode-0 连续发包时长
  int auto_clear_attempts_ = 0;  // 0 = 不自动清障(安全优先)
  // ---- ENABLE 温和化参数(事故后新增:防脏数据/错误位姿猛拉) ----
  int enable_ramp_ms_ = 2000;            // 增益从起点爬升到设定值的时长
  double enable_ramp_start_frac_ = 0.3;  // 爬升起点占 kp/kd 的比例(0.3:kp=60 时起步 18,重力下垂 <0.15 rad)
  double enable_max_step_ = 0.01;        // 保持目标每 tick 向实测逼近的最大步长 (rad)
  double enable_drift_max_ = 0.5;        // ENABLE 期间实测偏离捕获位姿的上限 (rad,kp>0 生效;容忍爬升期重力下垂)
  double pose_sanity_margin_ = 0.05;     // PROBE 捕获位姿允许超出 URDF 限位的余量 (rad)
  int port_guard_fd_ = -1;               // 串口独占守卫 fd(TIOCEXCL)
  // ---- 读数跳变过滤(2026-11-26 J1 多圈编码器偶发复位事件后新增) ----
  double read_jump_max_ = 1.0;   // 单 tick 读数跳变上限 (rad;100Hz 下 1.0≈100 rad/s,远超物理极限)
  double read_dq_max_ = 100.0;   // dq 绝对值上限 (rad/s;臂最大 37.7)
  int jump_confirm_ticks_ = 3;   // 连续 N 次一致才接受跳变读数(多圈复位后恢复的真实位置)
  std::array<double, kArmJointCount> jump_candidate_{};  // 跳变候选值(NaN=无候选)
  std::array<int, kArmJointCount> jump_streak_{};        // 候选连续一致计数
  std::array<double, kArmJointCount> limit_lower_;
  std::array<double, kArmJointCount> limit_upper_;
  // 读数补偿:q_out = comp_dir * q_bus + comp_offset(本台臂编码器零点与
  // 出厂模板标定不一致时的补偿;官方给出对接方式后可归零,不动官方标定文件)
  std::array<double, kArmJointCount> comp_dir_;
  std::array<double, kArmJointCount> comp_offset_;

  // ---- 状态机 ----
  std::atomic<ArmPhase> phase_{ArmPhase::IDLE};
  std::atomic<bool> clear_requested_{false};
  std::chrono::steady_clock::time_point phase_start_;
  std::chrono::steady_clock::time_point last_open_attempt_{};  // 串口重试节流
  std::array<float, kArmJointCount> q0_{};                 // ENABLE 起始位姿(实测,补偿后 URDF 系)
  std::array<double, kArmJointCount> last_sent_cmd_;       // 超前限幅基准
  std::array<double, kArmJointCount> last_seen_cmd_;       // 陈旧命令判据
  std::array<uint32_t, kArmJointCount> last_error_code_{}; // 错误码去重
  bool commands_validated_ = false;
  std::chrono::steady_clock::time_point last_fault_log_tp_{};  // FAULT 日志节流
  int auto_clear_count_ = 0;

  // ---- clear_faults 服务(独立线程,只写原子标志) ----
  rclcpp::Node::SharedPtr service_node_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_service_;
  rclcpp::executors::SingleThreadedExecutor service_executor_;
  std::thread service_thread_;
  std::atomic<bool> service_running_{false};
};
} // namespace r1_arm_hardware
