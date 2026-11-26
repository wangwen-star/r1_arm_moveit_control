// MotorProtocol.h 必须最先包含:其内部声明 MotorProtocol::M_PI 常量,
// 若先包含 <cmath>(ROS2 构建启用 GNU 扩展会定义 M_PI 宏)会导致编译错误
#include "MotorProtocol.h"
#include "r1_arm_hardware/r1_arm_hardware.hpp"
#include <pluginlib/class_list_macros.hpp>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>

// 串口独占防护(/proc 扫描 + TIOCEXCL)
#include <dirent.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>

#ifndef R1_SDK_ROOT_PATH
#define R1_SDK_ROOT_PATH "/home/wangwenyu/桌面/X86_SDK/r1_arm"
#endif

namespace r1_arm_hardware
{

R1ArmHardware::R1ArmHardware()
{
    last_sent_cmd_.fill(std::numeric_limits<double>::quiet_NaN());
    last_seen_cmd_.fill(std::numeric_limits<double>::quiet_NaN());
    last_error_code_.fill(0);
    jump_candidate_.fill(std::numeric_limits<double>::quiet_NaN());
    jump_streak_.fill(0);
}

R1ArmHardware::~R1ArmHardware()
{
    // 停止服务线程(先取消 executor 再 join,避免 rclcpp 关闭后悬空)
    if (service_running_.exchange(false))
    {
        service_executor_.cancel();
        if (service_thread_.joinable())
        {
            service_thread_.join();
        }
    }
    // ArmMotorBus 析构会自动 close()(内部先发软停止包)
    // 关闭守卫 fd,解除 TIOCEXCL(进程退出时内核也会自动清理)
    unlockPortExclusive();
}

// ---- 发包辅助(与 SDK test_arm_lowlevel 一致) ----

void R1ArmHardware::sendStopCmds()
{
    for (int i = 0; i < kArmJointCount; ++i)
    {
        cached_cmds_[i].mode = 0;
        cached_cmds_[i].timed_out = false;
        cached_cmds_[i].q = 0.f;
        cached_cmds_[i].dq = 0.f;
        cached_cmds_[i].tau = 0.f;
        cached_cmds_[i].kp = 0.f;
        cached_cmds_[i].kd = 0.f;
    }
    motor_bus_->step(cached_cmds_);
}

void R1ArmHardware::sendZeroTorqueCmds()
{
    // 零力矩 FOC:电机只回馈不出力(反馈探测用)
    for (int i = 0; i < kArmJointCount; ++i)
    {
        cached_cmds_[i].mode = 1;
        cached_cmds_[i].timed_out = false;
        cached_cmds_[i].q = 0.f;
        cached_cmds_[i].dq = 0.f;
        cached_cmds_[i].tau = 0.f;
        cached_cmds_[i].kp = 0.f;
        cached_cmds_[i].kd = 0.f;
    }
    motor_bus_->step(cached_cmds_);
}

void R1ArmHardware::sendFocHold(const std::array<float, kArmJointCount> & q, float kp, float kd)
{
    for (int i = 0; i < kArmJointCount; ++i)
    {
        cached_cmds_[i].mode = 1;
        cached_cmds_[i].timed_out = false;
        cached_cmds_[i].q = q[i];
        cached_cmds_[i].dq = 0.f;
        cached_cmds_[i].tau = 0.f;
        cached_cmds_[i].kp = kp;
        cached_cmds_[i].kd = kd;
    }
    motor_bus_->step(cached_cmds_);
}

void R1ArmHardware::enterFault(const std::string & reason)
{
    RCLCPP_ERROR(rclcpp::get_logger("R1ArmHardware"),
                 "进入 FAULT 状态: %s(持续发送 mode=0 停止包)", reason.c_str());
    phase_.store(ArmPhase::FAULT);
    phase_start_ = std::chrono::steady_clock::now();
    last_fault_log_tp_ = std::chrono::steady_clock::time_point{};  // 立即触发第一条节流日志
}

// ---- 参数读取 ----

std::string R1ArmHardware::getParamStr(const std::string & name, const std::string & def) const
{
    try
    {
        return info_.hardware_parameters.at(name);
    }
    catch (const std::out_of_range &)
    {
        return def;
    }
}

double R1ArmHardware::getParamDouble(const std::string & name, double def) const
{
    try
    {
        return std::stod(info_.hardware_parameters.at(name));
    }
    catch (...)
    {
        return def;
    }
}

int R1ArmHardware::getParamInt(const std::string & name, int def) const
{
    try
    {
        return std::stoi(info_.hardware_parameters.at(name));
    }
    catch (...)
    {
        return def;
    }
}

// ---- 串口独占防护(2026-10-28 撞限位事故教训) ----

pid_t R1ArmHardware::findPortOwner() const
{
    // 按设备号(st_rdev)匹配:兼容 /dev/ttyR1 → /dev/ttyUSB0 之类的符号链接别名。
    // 只做只读扫描,不发送任何信号。
    struct stat port_st{};
    if (::stat(port_.c_str(), &port_st) != 0)
    {
        return 0;  // 设备文件不存在(臂未接入/未拔插到位),不视为占用
    }
    if (!S_ISCHR(port_st.st_mode))
    {
        return 0;  // 不是字符设备(串口才可能是字符设备),跳过检查
    }
    const dev_t rdev = port_st.st_rdev;
    const pid_t self = ::getpid();

    DIR *proc = ::opendir("/proc");
    if (!proc)
    {
        return -1;
    }
    pid_t owner = -1;
    struct dirent *de = nullptr;
    while ((de = ::readdir(proc)) != nullptr)
    {
        // 只关心纯数字目录(PID/TID);注意线程目录也会出现,fds 表与主进程共享,无妨
        if (de->d_name[0] < '0' || de->d_name[0] > '9')
        {
            continue;
        }
        const pid_t pid = static_cast<pid_t>(std::atoi(de->d_name));
        if (pid == self)
        {
            continue;
        }
        const std::string fd_dir = std::string("/proc/") + de->d_name + "/fd";
        DIR *fd_proc = ::opendir(fd_dir.c_str());
        if (!fd_proc)
        {
            continue;  // 无权限(其他用户进程)或进程已退出
        }
        struct dirent *fde = nullptr;
        while ((fde = ::readdir(fd_proc)) != nullptr)
        {
            if (fde->d_name[0] == '.')
            {
                continue;
            }
            const std::string link = fd_dir + "/" + fde->d_name;
            char buf[256];
            const ssize_t n = ::readlink(link.c_str(), buf, sizeof(buf) - 1);
            if (n <= 0)
            {
                continue;
            }
            buf[n] = '\0';
            struct stat fd_st{};
            if (::stat(buf, &fd_st) != 0)
            {
                continue;  // 已删除文件/socket/管道等伪链接
            }
            if (S_ISCHR(fd_st.st_mode) && fd_st.st_rdev == rdev)
            {
                owner = pid;
                break;
            }
        }
        ::closedir(fd_proc);
        if (owner >= 0)
        {
            break;
        }
    }
    ::closedir(proc);
    return owner;
}

std::string R1ArmHardware::readProcComm(pid_t pid) const
{
    std::ifstream f("/proc/" + std::to_string(pid) + "/comm");
    std::string s;
    if (f.good())
    {
        std::getline(f, s);
    }
    return s.empty() ? "?" : s;
}

bool R1ArmHardware::lockPortExclusive()
{
    if (port_guard_fd_ >= 0)
    {
        return true;  // 已锁定
    }
    // 用独立的守卫 fd 设置 TIOCEXCL:终端独占锁是全局的(作用于 tty),
    // 设置后其他任何进程 open() 本端口都会 EBUSY 失败;
    // SDK 已打开的 fd 不受影响。守卫 fd 自己不读写,不会抢数据。
    const int fd = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0)
    {
        return false;
    }
    if (::ioctl(fd, TIOCEXCL) != 0)
    {
        ::close(fd);
        return false;
    }
    port_guard_fd_ = fd;
    return true;
}

void R1ArmHardware::unlockPortExclusive()
{
    if (port_guard_fd_ >= 0)
    {
        ::close(port_guard_fd_);
        port_guard_fd_ = -1;
    }
}

// ---- 生命周期 ----

hardware_interface::CallbackReturn R1ArmHardware::on_init(const hardware_interface::HardwareInfo & info)
{
    info_ = info;
    hw_joint_commands_.resize(kArmJointCount, 0.0);
    hw_joint_states_.resize(kArmJointCount, 0.0);
    hw_joint_states_vel_.resize(kArmJointCount, 0.0);

    // ---- 参数解析(全部带默认值回退) ----
    port_ = getParamStr("serial_port", "/dev/ttyUSB0");
    baud_ = getParamInt("baud_rate", 6000000);
    calib_path_ = getParamStr("calib_path", "");
    kp_ = getParamDouble("kp", 60.0);
    kd_ = getParamDouble("kd", 0.5);
    max_lead_ = getParamDouble("max_lead", 0.15);
    probe_timeout_ms_ = getParamInt("probe_timeout_ms", 3000);
    enable_timeout_ms_ = getParamInt("enable_timeout_ms", 1000);
    stop_stream_ms_ = getParamInt("stop_stream_ms", 500);
    auto_clear_attempts_ = getParamInt("auto_clear_attempts", 0);
    enable_ramp_ms_ = std::max(0, getParamInt("enable_ramp_ms", 2000));
    enable_ramp_start_frac_ = std::clamp(getParamDouble("enable_ramp_start_frac", 0.3), 0.0, 1.0);
    enable_max_step_ = std::max(0.0, getParamDouble("enable_max_step", 0.01));
    enable_drift_max_ = std::max(0.0, getParamDouble("enable_drift_max", 0.5));
    pose_sanity_margin_ = std::max(0.0, getParamDouble("pose_sanity_margin", 0.05));
    read_jump_max_ = std::max(0.0, getParamDouble("read_jump_max", 1.0));
    read_dq_max_ = std::max(0.0, getParamDouble("read_dq_max", 100.0));
    jump_confirm_ticks_ = std::max(1, getParamInt("jump_confirm_ticks", 3));

    // calib_path 强制绝对化(避免 SDK 的 CWD 相对路径陷阱)
    if (calib_path_.empty())
    {
        calib_path_ = std::string(R1_SDK_ROOT_PATH) + "/config/joint_calib.txt";
    }
    else if (calib_path_[0] != '/')
    {
        calib_path_ = std::string(R1_SDK_ROOT_PATH) + "/" + calib_path_;
    }

    // ---- 读数补偿参数(7 个数空格分隔;默认不补偿) ----
    {
        auto parse_7 = [](const std::string & text,
                          std::array<double, kArmJointCount> & out) {
            std::istringstream iss(text);
            double v = 0.0;
            int n = 0;
            while (iss >> v && n < kArmJointCount)
            {
                out[n++] = v;
            }
            return n == kArmJointCount;
        };
        comp_dir_.fill(1.0);
        comp_offset_.fill(0.0);
        const std::string dir_text = getParamStr("joint_dir", "");
        const std::string off_text = getParamStr("joint_offset", "");
        if (!dir_text.empty())
        {
            if (!parse_7(dir_text, comp_dir_))
            {
                RCLCPP_WARN(rclcpp::get_logger("R1ArmHardware"),
                            "joint_dir 参数格式错误,使用默认 +1");
                comp_dir_.fill(1.0);
            }
        }
        if (!off_text.empty())
        {
            if (!parse_7(off_text, comp_offset_))
            {
                RCLCPP_WARN(rclcpp::get_logger("R1ArmHardware"),
                            "joint_offset 参数格式错误,使用默认 0");
                comp_offset_.fill(0.0);
            }
        }
        RCLCPP_INFO(rclcpp::get_logger("R1ArmHardware"),
                    "读数补偿: dir=[%.0f %.0f %.0f %.0f %.0f %.0f %.0f] "
                    "offset=[%.3f %.3f %.3f %.3f %.3f %.3f %.3f]",
                    comp_dir_[0], comp_dir_[1], comp_dir_[2], comp_dir_[3],
                    comp_dir_[4], comp_dir_[5], comp_dir_[6],
                    comp_offset_[0], comp_offset_[1], comp_offset_[2],
                    comp_offset_[3], comp_offset_[4], comp_offset_[5],
                    comp_offset_[6]);
    }

    // ---- 关节限幅:优先 xacro 参数,缺失回退硬编码表(与 URDF 一致) ----
    {
        // lower: -2.723, 0, -0.349, 0, -2.269, -0.524, -1.571
        // upper:  2.723, 3.1416, 3.491, 3.1415, 2.269, 0.524, 1.571
        const std::array<double, kArmJointCount> kDefaultLower =
            {-2.723, 0.0, -0.349, 0.0, -2.269, -0.524, -1.571};
        const std::array<double, kArmJointCount> kDefaultUpper =
            {2.723, 3.1416, 3.491, 3.1415, 2.269, 0.524, 1.571};
        limit_lower_ = kDefaultLower;
        limit_upper_ = kDefaultUpper;

        auto parse_limits = [](const std::string & text,
                               std::array<double, kArmJointCount> & out) {
            std::istringstream iss(text);
            double v = 0.0;
            int n = 0;
            while (iss >> v && n < kArmJointCount)
            {
                out[n++] = v;
            }
            return n == kArmJointCount;
        };

        const std::string lower_text = getParamStr("limit_lower", "");
        const std::string upper_text = getParamStr("limit_upper", "");
        if (!lower_text.empty() && !upper_text.empty())
        {
            if (parse_limits(lower_text, limit_lower_) &&
                parse_limits(upper_text, limit_upper_))
            {
                RCLCPP_INFO(rclcpp::get_logger("R1ArmHardware"),
                            "关节限幅取自 xacro 参数");
            }
            else
            {
                RCLCPP_WARN(rclcpp::get_logger("R1ArmHardware"),
                            "limit_lower/limit_upper 参数格式错误,回退硬编码限位表");
            }
        }
    }

    // ---- clear_faults 服务(独立线程;回调只写原子标志,不动总线) ----
    service_node_ = rclcpp::Node::make_shared("r1_arm_hardware");
    clear_service_ = service_node_->create_service<std_srvs::srv::Trigger>(
        "~/clear_faults",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
            clear_requested_.store(true);
            resp->success = true;
            resp->message = "清障指令已提交,将在下一控制周期发送";
        });
    service_executor_.add_node(service_node_);
    service_running_.store(true);
    service_thread_ = std::thread([this]() { service_executor_.spin(); });

    RCLCPP_INFO(rclcpp::get_logger("R1ArmHardware"),
                "on_init 完成: port=%s, baud=%d, kp=%.1f, kd=%.2f, calib=%s",
                port_.c_str(), baud_, kp_, kd_, calib_path_.c_str());
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> R1ArmHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
        state_interfaces.emplace_back(
            info_.joints[i].name,
            hardware_interface::HW_IF_POSITION,
            &hw_joint_states_[i]);
        state_interfaces.emplace_back(
            info_.joints[i].name,
            hardware_interface::HW_IF_VELOCITY,
            &hw_joint_states_vel_[i]);
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> R1ArmHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
        command_interfaces.emplace_back(
            info_.joints[i].name,
            hardware_interface::HW_IF_POSITION,
            &hw_joint_commands_[i]);
    }
    return command_interfaces;
}

hardware_interface::CallbackReturn R1ArmHardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
    // 注意:on_configure 必须始终返回 SUCCESS——Humble 的 controller_manager 在
    // 构造函数里配置所有硬件组件,任何 ERROR 都会让整个节点抛异常崩溃。
    // 串口打不开时记录错误并把打开推迟到 write() 里周期重试(支持真臂热插拔)。

    // 标定文件预检:SDK 的 open() 在标定失败时会直接 std::exit(1),
    // 必须先做文件存在检查 + loadJointCalibration() 预检(返回 bool 不退出)
    {
        std::ifstream f(calib_path_);
        if (!f.good())
        {
            RCLCPP_ERROR(rclcpp::get_logger("R1ArmHardware"),
                         "标定文件不存在或不可读: %s(将在 write() 中周期重试)",
                         calib_path_.c_str());
            return hardware_interface::CallbackReturn::SUCCESS;
        }
    }

    if (!motor_bus_)
    {
        motor_bus_ = std::make_unique<ArmMotorBus>();
    }
    if (!motor_bus_->loadJointCalibration(calib_path_))
    {
        RCLCPP_ERROR(rclcpp::get_logger("R1ArmHardware"),
                     "标定文件解析失败(需要 7 行 joint dir scale offset): %s",
                     calib_path_.c_str());
        return hardware_interface::CallbackReturn::SUCCESS;
    }
    // ---- 串口占用检查:发现其他进程已打开本端口则拒绝打开(write() 里每 2s 重试)。
    // 2026-10-28 撞限位事故的直接原因就是两个程序争用同一串口、数据打架。
    {
        const pid_t owner = findPortOwner();
        if (owner > 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("R1ArmHardware"),
                         "串口 %s 已被进程 %d(%s)占用,拒绝打开;请先停止该程序"
                         "(插件将每 2s 重试)",
                         port_.c_str(), static_cast<int>(owner),
                         readProcComm(owner).c_str());
            return hardware_interface::CallbackReturn::SUCCESS;
        }
    }
    // 此时标定必成功,open() 内部不会再触发 std::exit
    // SDK 的 SerialPort::open 在设备不存在/权限不足时会抛 std::runtime_error,必须接住
    bool opened = false;
    try
    {
        opened = motor_bus_->open(port_, baud_, calib_path_);
    }
    catch (const std::exception & e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("R1ArmHardware"),
                     "打开串口异常: %s(设备: %s,将在 write() 中每 2s 重试)",
                     e.what(), port_.c_str());
        return hardware_interface::CallbackReturn::SUCCESS;
    }
    if (!opened)
    {
        RCLCPP_ERROR(rclcpp::get_logger("R1ArmHardware"),
                     "打开串口失败: %s @ %d(设备不存在或权限不足,将在 write() 中每 2s 重试)",
                     port_.c_str(), baud_);
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // ---- 打开成功后立刻加 TIOCEXCL 独占锁:此后其他任何进程都打不开本端口,
    // 从根上杜绝"另一个程序半路插进来争抢串口"的事故场景 ----
    if (lockPortExclusive())
    {
        RCLCPP_INFO(rclcpp::get_logger("R1ArmHardware"),
                    "串口已加独占锁(TIOCEXCL),其他进程将无法打开 %s", port_.c_str());
    }
    else
    {
        RCLCPP_WARN(rclcpp::get_logger("R1ArmHardware"),
                    "串口独占锁设置失败(继续运行,依赖 /proc 占用检查兜底)");
    }

    RCLCPP_INFO(rclcpp::get_logger("R1ArmHardware"),
                "串口打开成功: %s @ %d, 标定: %s(configure 阶段不发包,电机保持脱力)",
                port_.c_str(), baud_, calib_path_.c_str());
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn R1ArmHardware::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
{
    phase_.store(ArmPhase::IDLE);
    if (motor_bus_ && motor_bus_->isOpen())
    {
        try
        {
            motor_bus_->stop();
            motor_bus_->close();
        }
        catch (const std::exception & e)
        {
            RCLCPP_WARN(rclcpp::get_logger("R1ArmHardware"),
                        "on_cleanup 关闭串口异常: %s", e.what());
        }
    }
    unlockPortExclusive();
    RCLCPP_INFO(rclcpp::get_logger("R1ArmHardware"), "on_cleanup: 串口已关闭");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn R1ArmHardware::on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/)
{
    // 兜底:确保电机停止、串口关闭(ArmMotorBus 析构也会 close)
    if (motor_bus_ && motor_bus_->isOpen())
    {
        phase_.store(ArmPhase::IDLE);
        try
        {
            motor_bus_->stop();
            motor_bus_->close();
        }
        catch (const std::exception & e)
        {
            RCLCPP_WARN(rclcpp::get_logger("R1ArmHardware"),
                        "on_shutdown 关闭串口异常: %s", e.what());
        }
    }
    unlockPortExclusive();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn R1ArmHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    // 注意:必须无条件返回 SUCCESS(Humble 的 controller_manager 在启动流程里
    // 会尝试把组件置为 active,任何 ERROR 都会让整个节点崩溃)。
    // 串口未打开时由 write() 的重试分支负责打开并重启 PROBE。

    // 只置位,不阻塞:PROBE/ENABLE 在 write() 里异步推进,
    // 保证 joint_state_broadcaster 从启动第 0 秒就正常广播
    q0_.fill(0.f);
    last_sent_cmd_.fill(std::numeric_limits<double>::quiet_NaN());
    last_seen_cmd_.fill(std::numeric_limits<double>::quiet_NaN());
    last_error_code_.fill(0);
    jump_candidate_.fill(std::numeric_limits<double>::quiet_NaN());
    jump_streak_.fill(0);
    commands_validated_ = false;
    auto_clear_count_ = 0;
    std::fill(hw_joint_commands_.begin(), hw_joint_commands_.end(), 0.0);

    phase_.store(ArmPhase::PROBE);
    phase_start_ = std::chrono::steady_clock::now();
    if (motor_bus_ && motor_bus_->isOpen())
    {
        RCLCPP_INFO(rclcpp::get_logger("R1ArmHardware"),
                    "on_activate: 进入 PROBE(零力矩 FOC 探测反馈,电机保持脱力)");
    }
    else
    {
        RCLCPP_WARN(rclcpp::get_logger("R1ArmHardware"),
                    "on_activate: 串口尚未打开,write() 将每 2s 重试(真臂接入后自动进入 PROBE)");
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn R1ArmHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    phase_.store(ArmPhase::IDLE);
    if (motor_bus_ && motor_bus_->isOpen())
    {
        try
        {
            // 仿 SDK shutdown:连续发 500ms mode=0 停止包,再 stop() 兜底
            const int ticks = std::max(1, stop_stream_ms_ / 20);
            for (int k = 0; k < ticks; ++k)
            {
                sendStopCmds();
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
            motor_bus_->stop();
            RCLCPP_INFO(rclcpp::get_logger("R1ArmHardware"),
                        "on_deactivate: 已发送 %d ms mode=0 停止流,电机脱力(请扶住手臂)",
                        stop_stream_ms_);
        }
        catch (const std::exception & e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("R1ArmHardware"),
                         "on_deactivate 停止流发送异常: %s", e.what());
        }
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

// ---- 实时循环 ----

hardware_interface::return_type R1ArmHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    if (!motor_bus_ || !motor_bus_->isOpen())
    {
        return hardware_interface::return_type::OK;
    }
    try
    {
        const auto st = motor_bus_->states();   // 总线已应用模板标定,再叠本台臂补偿
        for (int i = 0; i < kArmJointCount; i++)
        {
            // 非法读数(NaN/Inf)保持上一帧:脏数据不广播,防止 TF/RViz 崩,
            // 也防止 write() 里的漂移保护被 NaN 干扰
            const double q = comp_dir_[i] * st[i].q + comp_offset_[i];
            const double dq = comp_dir_[i] * st[i].dq;  // SDK 直接给出 dq
            if (std::isfinite(dq) && std::fabs(dq) <= read_dq_max_)
            {
                hw_joint_states_vel_[i] = dq;
            }
            if (!std::isfinite(q))
            {
                continue;  // 保持上一帧
            }
            // ---- 读数跳变过滤(2026-11-26 J1 多圈编码器偶发复位事件后新增) ----
            // 反馈包有 CRC 校验,坏包到不了这里;但编码器本身可能偶发报出错误值
            // (多圈计数复位,读数瞬间跳 2.7 rad)。单 tick 内超过 read_jump_max_
            // 的位移物理上不可能(100 rad/s 级),视为跳变:连续 jump_confirm_ticks_
            // 次一致才接受为真实位置,否则保持上一帧。PROBE 捕获/ENABLE 种命令
            // 用的都是这里的过滤值,脏读数从此进不了控制回路。
            const double delta = std::fabs(q - hw_joint_states_[i]);
            if (delta > read_jump_max_)
            {
                if (std::isfinite(jump_candidate_[i]) &&
                    std::fabs(q - jump_candidate_[i]) < 1e-3)
                {
                    if (++jump_streak_[i] >= jump_confirm_ticks_)
                    {
                        RCLCPP_WARN(rclcpp::get_logger("R1ArmHardware"),
                                    "J%d 读数跳变 %.3f -> %.3f 连续确认,接受为新位置"
                                    "(疑似多圈编码器复位后恢复)",
                                    i + 1, hw_joint_states_[i], q);
                        hw_joint_states_[i] = q;
                        jump_candidate_[i] = std::numeric_limits<double>::quiet_NaN();
                        jump_streak_[i] = 0;
                    }
                }
                else
                {
                    jump_candidate_[i] = q;
                    jump_streak_[i] = 1;
                }
                // 跳变未确认:保持上一帧,不更新
            }
            else
            {
                jump_candidate_[i] = std::numeric_limits<double>::quiet_NaN();
                jump_streak_[i] = 0;
                hw_joint_states_[i] = q;
            }
        }
    }
    catch (const std::exception & e)
    {
        static rclcpp::Clock throttle_clock(RCL_STEADY_TIME);
        RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("R1ArmHardware"),
                              throttle_clock, 1000,
                              "read() 异常: %s", e.what());
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type R1ArmHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    auto logger = rclcpp::get_logger("R1ArmHardware");

    // 串口未打开:每 2s 重试一次(configure 失败时走这里,支持真臂热插拔)
    if (!motor_bus_ || !motor_bus_->isOpen())
    {
        const auto now = std::chrono::steady_clock::now();
        if (now - last_open_attempt_ >= std::chrono::seconds(2))
        {
            last_open_attempt_ = now;
            if (!motor_bus_)
            {
                motor_bus_ = std::make_unique<ArmMotorBus>();
            }
            // open() 前必须先预检标定(SDK 在标定失败时会 std::exit)
            if (!std::ifstream(calib_path_).good() ||
                !motor_bus_->loadJointCalibration(calib_path_))
            {
                RCLCPP_WARN(logger, "标定文件不可用,2s 后重试: %s", calib_path_.c_str());
                return hardware_interface::return_type::OK;
            }
            // 串口占用检查:被其他进程占用时跳过本次尝试,等对方释放
            const pid_t owner = findPortOwner();
            if (owner > 0)
            {
                RCLCPP_WARN(logger,
                            "串口 %s 被进程 %d(%s)占用,2s 后重试;请停止该程序",
                            port_.c_str(), static_cast<int>(owner),
                            readProcComm(owner).c_str());
                return hardware_interface::return_type::OK;
            }
            bool opened = false;
            try
            {
                opened = motor_bus_->open(port_, baud_, calib_path_);
            }
            catch (const std::exception & e)
            {
                RCLCPP_WARN(logger, "打开串口异常: %s(2s 后重试)", e.what());
                return hardware_interface::return_type::OK;
            }
            if (!opened)
            {
                RCLCPP_WARN(logger, "串口仍不可用: %s(2s 后重试)", port_.c_str());
                return hardware_interface::return_type::OK;
            }
            if (!lockPortExclusive())
            {
                RCLCPP_WARN(logger, "串口独占锁设置失败(继续运行,依赖占用检查兜底)");
            }
            RCLCPP_INFO(logger, "串口已打开: %s @ %d,重新进入 PROBE",
                        port_.c_str(), baud_);
            phase_.store(ArmPhase::PROBE);
            phase_start_ = now;
        }
        return hardware_interface::return_type::OK;
    }

    try
    {
    // 消费 clear_faults 服务请求(一轮 = 7 个清障包,不在 RT 线程里 sleep)
    if (clear_requested_.exchange(false))
    {
        motor_bus_->clearFaults();
        RCLCPP_INFO(logger, "已发送清障指令(clearFaults)");
        if (phase_.load() == ArmPhase::FAULT)
        {
            last_error_code_.fill(0);   // 清障后重新记录错误码
            phase_.store(ArmPhase::PROBE);
            phase_start_ = std::chrono::steady_clock::now();
            RCLCPP_INFO(logger, "清障后重新进入 PROBE");
        }
    }

    const auto st = motor_bus_->states();
    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                std::chrono::steady_clock::now() - phase_start_)
                                .count();

    switch (phase_.load())
    {
        case ArmPhase::PROBE:
        {
            // 零力矩 FOC 探测:电机只回馈不出力
            sendZeroTorqueCmds();
            if (motor_bus_->allFeedbackReady())
            {
                // 捕获用 read() 里过滤后的读数(跳变防护后):先确认过滤器处于
                // 干净状态(无待确认的跳变候选),否则等下一 tick——宁可多等,
                // 绝不把多圈编码器复位的假读数捕获进 q0(2026-11-26 事件)。
                bool filter_busy = false;
                for (int i = 0; i < kArmJointCount; i++)
                {
                    if (jump_streak_[i] > 0)
                    {
                        filter_busy = true;
                        break;
                    }
                }
                if (filter_busy)
                {
                    break;  // 等 PROBE 超时兜底前让过滤器先收敛
                }
                // 捕获位姿并做合理性校验:
                // 越界/NaN 说明标定不匹配、编码器异常或数据被污染(串口被争抢),
                // 一律拒绝使能进 FAULT——宁可不动,绝不按脏位姿出力
                // (2026-10-28 撞限位事故教训)。
                bool sane = true;
                for (int i = 0; i < kArmJointCount; i++)
                {
                    q0_[i] = hw_joint_states_[i];
                    if (!std::isfinite(q0_[i]) ||
                        q0_[i] < limit_lower_[i] - pose_sanity_margin_ ||
                        q0_[i] > limit_upper_[i] + pose_sanity_margin_)
                    {
                        sane = false;
                    }
                }
                if (!sane)
                {
                    std::ostringstream oss;
                    oss << "PROBE 捕获位姿不合理,拒绝使能:";
                    for (int i = 0; i < kArmJointCount; i++)
                    {
                        oss << " J" << (i + 1) << "=" << q0_[i];
                    }
                    oss << " (疑似标定不匹配/编码器异常/数据污染,"
                           "请检查标定文件与串口占用)";
                    enterFault(oss.str());
                    break;
                }
                RCLCPP_INFO(logger,
                            "反馈全部就绪,捕获位姿 q0=[%.3f %.3f %.3f %.3f %.3f %.3f %.3f],"
                            "进入 ENABLE(增益渐进爬升)",
                            q0_[0], q0_[1], q0_[2], q0_[3], q0_[4], q0_[5], q0_[6]);
                phase_.store(ArmPhase::ENABLE);
                phase_start_ = std::chrono::steady_clock::now();
            }
            else if (elapsed_ms >= probe_timeout_ms_)
            {
                std::string missing;
                const auto ready = motor_bus_->feedbackReady();
                for (int i = 0; i < kArmJointCount; i++)
                {
                    if (!ready[i])
                    {
                        missing += " J" + std::to_string(i + 1);
                    }
                }
                enterFault("PROBE 超时,以下关节无反馈:" + missing +
                           "(检查供电/接线/适配器是否支持 6Mbps)");
            }
            break;
        }
        case ArmPhase::ENABLE:
        {
            // ---- ENABLE 温和化(2026-10-28 事故后):三层防猛拉 ----
            // ① 增益渐进:kp/kd 从 enable_ramp_start_frac 在 enable_ramp_ms_ 内
            //    线性爬升到设定值,初期只出很小的力;
            // ② 目标步进限幅:保持目标每 tick 最多向实测位姿逼近 enable_max_step_,
            //    即使捕获位姿与实测有偏差也只能慢慢靠拢;
            // ③ 漂移保护:实测偏离捕获位姿超过 enable_drift_max_ 立即 FAULT。
            // kp<=0(零力矩镜像模式)时增益恒为 0,②③ 的爬升/漂移检查自动跳过,
            // 保持事故前"零力矩只做 RViz 镜像"的行为。
            const double ramp_frac = (kp_ <= 0.0)
                ? 1.0
                : std::clamp(elapsed_ms / static_cast<double>(enable_ramp_ms_), 0.0, 1.0);
            const double gain_frac = (kp_ <= 0.0)
                ? 0.0
                : enable_ramp_start_frac_ + (1.0 - enable_ramp_start_frac_) * ramp_frac;
            const float kp_now = static_cast<float>(kp_ * gain_frac);
            const float kd_now = static_cast<float>(kd_ * gain_frac);

            // 保持目标:向 q0_ 靠拢,但每 tick 步进限幅;换算总线系后发包
            std::array<float, kArmJointCount> hold_q_bus{};
            for (int i = 0; i < kArmJointCount; i++)
            {
                double target = q0_[i];
                if (std::isfinite(hw_joint_states_[i]))
                {
                    target = std::clamp(target,
                                        hw_joint_states_[i] - enable_max_step_,
                                        hw_joint_states_[i] + enable_max_step_);
                }
                hold_q_bus[i] = static_cast<float>(
                    (target - comp_offset_[i]) / comp_dir_[i]);
            }
            sendFocHold(hold_q_bus, kp_now, kd_now);

            bool all_foc = true;
            for (int i = 0; i < kArmJointCount; i++)
            {
                if (st[i].mode != 1 || st[i].error_code != 0)
                {
                    all_foc = false;
                    break;
                }
            }

            // ③ 漂移保护(仅出力模式下生效)
            bool hold_ok = all_foc;
            if (hold_ok && kp_ > 0.0)
            {
                for (int i = 0; i < kArmJointCount; i++)
                {
                    if (std::fabs(hw_joint_states_[i] - q0_[i]) > enable_drift_max_)
                    {
                        std::ostringstream oss;
                        oss << "ENABLE 漂移超限 J" << (i + 1) << ": 实测 "
                            << hw_joint_states_[i] << " 偏离捕获位姿 " << q0_[i]
                            << " 超过 " << enable_drift_max_
                            << " rad(可能被人为拨动或数据异常)";
                        enterFault(oss.str());
                        hold_ok = false;
                        break;
                    }
                }
            }

            if (hold_ok && ramp_frac >= 1.0)
            {
                // 关键:把实测位姿种入命令(补偿后 URDF 系,与 read() 输出同源),
                // 防止 JTC 首帧跳变——不再用 q0_,实测永远比捕获值更可信
                for (int i = 0; i < kArmJointCount; i++)
                {
                    hw_joint_commands_[i] = hw_joint_states_[i];
                    last_sent_cmd_[i] = hw_joint_commands_[i];
                    last_seen_cmd_[i] = hw_joint_commands_[i];
                }
                commands_validated_ = false;
                RCLCPP_INFO(logger,
                            "全部电机进入 FOC 且增益爬升完成,进入 READY(开始接受轨迹指令)");
                phase_.store(ArmPhase::READY);
                phase_start_ = std::chrono::steady_clock::now();
            }
            else if (elapsed_ms >= enable_timeout_ms_ + enable_ramp_ms_)
            {
                std::ostringstream oss;
                for (int i = 0; i < kArmJointCount; i++)
                {
                    oss << "J" << (i + 1)
                        << "(mode=" << static_cast<int>(st[i].mode)
                        << ",to=" << static_cast<int>(st[i].timeout)
                        << ",err=0x" << std::hex << st[i].error_code << std::dec
                        << ") ";
                }
                enterFault("ENABLE 超时(含增益爬升),电机未全部进入 FOC: " + oss.str() +
                           "可先运行 SDK --clear/--reset 或断电重启");
            }
            break;
        }
        case ArmPhase::READY:
        {
            // ---- 组装命令(四重保护) ----
            std::array<float, kArmJointCount> q{};
            bool any_moved = false;
            for (int i = 0; i < kArmJointCount; i++)
            {
                double cmd = hw_joint_commands_[i];
                // ① NaN/无效(JTC 未接管)→ 保持实测位姿(补偿后 URDF 系)
                if (!std::isfinite(cmd))
                {
                    cmd = hw_joint_states_[i];
                }
                // ② URDF 限幅
                cmd = std::clamp(cmd, limit_lower_[i], limit_upper_[i]);
                // ③ 每 tick 超前限幅(防首帧跳变)
                if (std::isfinite(last_sent_cmd_[i]))
                {
                    cmd = std::clamp(cmd, last_sent_cmd_[i] - max_lead_,
                                     last_sent_cmd_[i] + max_lead_);
                }
                // ④ 陈旧保持判据:命令恒定且与实测相差 >0.2 rad = 不可信,忽略
                if (!commands_validated_ &&
                    std::fabs(cmd - hw_joint_states_[i]) > 0.2)
                {
                    if (std::isfinite(last_seen_cmd_[i]) &&
                        std::fabs(cmd - last_seen_cmd_[i]) < 1e-4)
                    {
                        cmd = hw_joint_states_[i];
                    }
                    else
                    {
                        any_moved = true;
                    }
                }
                last_seen_cmd_[i] = cmd;
                last_sent_cmd_[i] = cmd;
                q[i] = static_cast<float>(cmd);
            }
            if (!commands_validated_ && any_moved)
            {
                commands_validated_ = true;
            }

            // 发送 FOC 位置包(mode=1, dq=tau=0, kp/kd)
            // 命令是补偿后 URDF 系,发给总线前必须反补偿回总线系,
            // 否则 step() 会按模板标定把电机算到错误位置(真臂会冲错方向)
            std::array<float, kArmJointCount> q_bus{};
            for (int i = 0; i < kArmJointCount; i++)
            {
                q_bus[i] = static_cast<float>(
                    (q[i] - comp_offset_[i]) / comp_dir_[i]);
            }
            sendFocHold(q_bus, static_cast<float>(kp_), static_cast<float>(kd_));

            // ---- ⑤ 故障监测 ----
            bool fault = false;
            for (int i = 0; i < kArmJointCount; i++)
            {
                if (st[i].mode != 1 || st[i].timeout != 0 || st[i].error_code != 0)
                {
                    if (st[i].error_code != 0 && st[i].error_code != last_error_code_[i])
                    {
                        RCLCPP_ERROR(logger, "J%d 错误码 0x%x: %s", i + 1,
                                     st[i].error_code,
                                     MotorProtocol::getErrorDescription(st[i].error_code).c_str());
                        last_error_code_[i] = st[i].error_code;
                    }
                    fault = true;
                }
            }
            if (fault)
            {
                enterFault("READY 运行中检测到电机故障(见上方错误码)");
            }
            break;
        }
        case ArmPhase::FAULT:
        {
            sendStopCmds();
            // 1Hz 节流日志;auto_clear_attempts>0 时自动清障重试(默认 0,人工干预)
            // 注意:节流必须用 steady_clock(controller_manager 传入的 time 是
            // steady 时钟源,与默认构造的 rclcpp::Time 相减会抛异常)
            const auto now_tp = std::chrono::steady_clock::now();
            if (now_tp - last_fault_log_tp_ >= std::chrono::seconds(1))
            {
                last_fault_log_tp_ = now_tp;
                if (auto_clear_attempts_ > 0 && auto_clear_count_ < auto_clear_attempts_)
                {
                    auto_clear_count_++;
                    RCLCPP_WARN(logger, "FAULT: 自动清障第 %d/%d 次",
                                auto_clear_count_, auto_clear_attempts_);
                    motor_bus_->clearFaults();
                    phase_.store(ArmPhase::PROBE);
                    phase_start_ = std::chrono::steady_clock::now();
                }
                else
                {
                    RCLCPP_WARN(logger,
                                "FAULT: 持续发送停止包。可调用 /r1_arm_hardware/clear_faults 服务"
                                "或重启系统恢复");
                }
            }
            break;
        }
        case ArmPhase::IDLE:
        default:
        {
            sendStopCmds();
            break;
        }
    }
    }
    catch (const std::exception & e)
    {
        // 串口写失败等异常:进入 FAULT 持续发停止包,避免异常逃逸到 controller_manager
        static rclcpp::Clock throttle_clock(RCL_STEADY_TIME);
        RCLCPP_ERROR_THROTTLE(logger, throttle_clock, 1000,
                              "write() 异常: %s", e.what());
        if (phase_.load() != ArmPhase::FAULT)
        {
            enterFault(std::string("write() 异常: ") + e.what());
        }
    }
    return hardware_interface::return_type::OK;
}

} // namespace r1_arm_hardware

PLUGINLIB_EXPORT_CLASS(
    r1_arm_hardware::R1ArmHardware,
    hardware_interface::SystemInterface)
