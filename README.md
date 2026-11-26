# R1-7a 机械臂 MoveIt2 集成工程

基于宇树(Unitree)R1-7a 七轴机械臂的 ROS 2 工程:自研 ros2_control 硬件插件 + MoveIt2 运动规划,实现**在 RViz 中规划轨迹并控制真实机械臂运动**,同时**实时读取电机角度将真臂位姿镜像显示到 RViz**。

```
┌───────────┐  FollowJointTrajectory   ┌──────────────────┐  位置指令   ┌──────────────────┐  FOC包(6Mbps) ┌──────┐
│  MoveIt2  │ ───────────────────────▶ │ joint_trajectory │ ─────────▶ │ r1_arm_hardware │ ────────────▶ │ R1-7a│
│ (move_group│ 规划/执行(RViz面板操作)   │ _controller      │  100Hz     │ (ros2_control   │ ◀──────────── │ 真臂 │
│  + RViz)  │                          └──────────────────┘            │  硬件插件)      │  电机反馈     └──────┘
└───────────┘                                                                │  │
      ▲                                                                     │  │  /joint_states(实测角度)
      │ 规划场景/白臂位姿                                          ┌─────────┘  ▼
      └────────────────────────────────────────────────────────── │ robot_state_publisher → TF → RViz 白臂镜像
```

## 目录结构

| 包 | 作用 |
|---|---|
| `r1_arm` | URDF 模型(mesh 来自 SolidWorks 导出)+ 带硬件插件的 ros2_control xacro + 显示用 launch |
| `r1_arm_hardware` | 自研 ros2_control 硬件插件(C++):状态机 `IDLE→PROBE→ENABLE→READY→FAULT`,带多重安全保护 |
| `r1_arm_moveit_config` | MoveIt 配置(SRDF/运动学/关节限幅)+ 真臂与仿真两套 launch |

## 环境要求

- **硬件**:宇树 R1-7a 机械臂(7 关节,无夹爪)、FTDI FT232H USB-485 适配器(`0403:6014`,支持 6Mbps)、48V 电源
- **软件**:Ubuntu 22.04 + ROS 2 Humble + MoveIt2 + ros2_control
- **宇树官方 X86_SDK**:本插件直接编译 SDK 源码(`arm_motor_bus.cpp`)并链接 `libmotor_controller.so`,**SDK 不随本仓库分发,需自行向宇树获取**

```bash
sudo apt install ros-humble-moveit ros-humble-ros2-control ros-humble-ros2-controllers \
                 ros-humble-joint-state-broadcaster ros-humble-joint-trajectory-controller \
                 ros-humble-xacro ros-humble-robot-state-publisher
```

## ⚠️ 下载后必改清单(3 处硬编码路径 + 1 处串口)

| 文件 | 位置 | 改什么 |
|---|---|---|
| `r1_arm_hardware/CMakeLists.txt` | 第 20 行 | `set(R1_SDK_ROOT "/home/wangwenyu/桌面/X86_SDK/r1_arm")` → 改成你的 SDK 路径 |
| `r1_arm_hardware/src/r1_arm_hardware.cpp` | 第 25 行 | `#define R1_SDK_ROOT_PATH ...` 兜底宏 → 同上(正常构建时 CMakeLists 的宏会覆盖它,保持两处一致) |
| `r1_arm/urdf/r1_arm_with_hardware.xacro` | `calib_path` 参数 | 第10行`/home/wangwenyu/桌面/X86_SDK/r1_arm/config/joint_calib.txt` → 你的 SDK 标定文件绝对路径 |
| `r1_arm/urdf/r1_arm_with_hardware.xacro` | `serial_port` 参数 | 默认 `/dev/ttyR1`(依赖下面的 udev 规则);不想装规则就改成 `/dev/ttyUSB0` |
| 各 `package.xml` / `r1_arm/setup.py` | maintainer/author | (可选)维护者姓名与邮箱 |

> 建议用 `grep -rn "wangwenyu" .` 全仓自查一遍,避免遗漏。

## 安装步骤

**1. 放入工作区**(本仓库内容即 ROS 工作区的 `src/` 内容):

```bash
mkdir -p ~/r1_workspace/src
# 把仓库内容克隆/解压到 ~/r1_workspace/src/ 下
```

**2. 安装 udev 规则**(把适配器固定为 `/dev/ttyR1`,免 root 访问):

```bash
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", MODE="0666", SYMLINK+="ttyR1"' \
  | sudo tee /etc/udev/rules.d/99-r1-arm.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

**3. 构建**:

```bash
cd ~/r1_workspace
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## 快速上手

### 真臂模式(规划 + 控制 + RViz 镜像)

```bash
ros2 launch r1_arm_moveit_config demo_hardware.launch.py
```

启动后日志依次出现:`串口打开成功` → `进入 PROBE`(零力矩探测,~0.1 秒)→ `捕获位姿 q0=[...]` → `进入 ENABLE`(增益 2 秒爬升)→ **`进入 READY`**。

到 READY 后,在 RViz 左侧 **MotionPlanning 面板**拖拽目标位姿 → **Plan** → **Execute**,真臂执行轨迹,白色机械臂实时镜像真臂位姿。

### 无硬件仿真(验证规划管线)

```bash
ros2 launch r1_arm_moveit_config demo.launch.py   # mock_components 假硬件
```

### 仅显示 URDF(拖动滑条看模型)

```bash
ros2 launch r1_arm display.launch.py
```

## 日常操作

```bash
# 暂停悬停(臂冻结在当前位姿,不坠落;再 activate 恢复)——比 Ctrl+C 优雅
ros2 lifecycle set /joint_trajectory_controller deactivate
ros2 lifecycle set /joint_trajectory_controller activate

# 故障恢复(FAULT 状态下电机已脱力,清障后自动重新 PROBE→ENABLE→READY)
ros2 service call /r1_arm_hardware/clear_faults std_srvs/srv/Trigger

# 脱离 ROS 直接读电机角度自检(注意必须在 bin/ 目录运行,SDK 标定路径是相对路径)
cd <SDK路径>/r1_arm/bin && ./test_arm_lowlevel --zero-torque --port /dev/ttyR1
```

## 关键参数(`r1_arm/urdf/r1_arm_with_hardware.xacro`)

| 参数 | 默认值 | 说明 |
|---|---|---|
| `kp` / `kd` | 60.0 / 0.5 | FOC 增益(宇树官方默认)。**FOC 位置模式无重力补偿**,低增益时臂会下垂(静差 = 重力矩/kp);建议阶梯调整,勿直接拉满 |
| `serial_port` / `baud_rate` | /dev/ttyR1 / 6000000 | 串口与波特率 |
| `calib_path` | (必改) | SDK 标定文件绝对路径 |
| `enable_ramp_ms` | 2000 | ENABLE 阶段增益爬升时长 |
| `enable_ramp_start_frac` | 0.3 | 爬升起点占 kp/kd 比例 |
| `enable_max_step` | 0.01 | 保持目标每 tick 向实测位姿逼近的最大步长(rad) |
| `enable_drift_max` | 0.5 | ENABLE 期间实测偏离捕获位姿超此值 → FAULT |
| `pose_sanity_margin` | 0.1 | PROBE 捕获位姿允许超出 URDF 限位的余量 |
| `read_jump_max` / `read_dq_max` / `jump_confirm_ticks` | 1.0 / 100 / 3 | 读数跳变过滤(编码器多圈计数偶发复位防护) |
| `max_lead` | 0.15 | 命令每 tick 超前限幅(rad) |
| `joint_dir` / `joint_offset` | 恒等 | 本台臂读数补偿(官方标定与 URDF 零位不一致时启用;7 个数空格分隔) |

## 安全须知(重要!)

1. **串口独占保护**:插件打开串口前检测其他进程占用(被占则拒绝打开),打开后加 TIOCEXCL 独占锁——**launch 运行时,其他程序(含 SDK 工具)无法再打开该串口,这是设计行为**。不要同时开两个程序操作真臂。
2. **启动流程**:PROBE 阶段是零力矩,臂会自由下垂;ENABLE 阶段增益 2 秒爬升后才托住臂。启动时请扶好臂或让它停在自然位姿。
3. **关机坠落**:Ctrl+C 后臂脱力坠落是硬件特性(电机通讯超时保护),收工前先把臂放平;中途暂停请用 lifecycle deactivate(见上)。
4. **FAULT = 已脱力**:任何 FAULT 日志出现时电机都在持续发停止包,先松手、查原因、再 `clear_faults` 或重启。
5. 首次恢复增益请阶梯递进(如 10/0.2 → 30/0.3 → 60/0.5),每一步验证悬停与小幅度运动。

## 故障排查

| 现象 | 原因与处理 |
|---|---|
| `标定文件不存在或不可读` | xacro 里 `calib_path` 路径错误,改成绝对路径 |
| `串口 ... 已被进程 N 占用,拒绝打开` | 有程序占着串口(`fuser /dev/ttyR1` 查看),停掉它;插件每 2s 自动重试 |
| `打开串口失败` | 设备未插/udev 规则未装/无权限;SDK 工具在 `bin/` 目录运行报标定错误是它的相对路径坑,与插件无关 |
| FAULT:`PROBE 超时,以下关节无反馈` | 电机供电/接线问题,或适配器不支持 6Mbps(需 FTDI 0403:6014) |
| FAULT:`捕获位姿不合理,拒绝使能` | 标定与本台臂不匹配,检查 joint_calib.txt 或 `joint_offset` 补偿 |
| FAULT:`ENABLE 漂移超限` | 启动时臂被拨动/读数跳变,等臂静止后 `clear_faults` |
| FAULT:`电机故障 错误码 0x...` | 日志中带电机错误码,查 SDK 错误码说明,必要时 `clear_faults` 或断电重启 |
| RViz 白臂不显示 | RViz 需要完整 MoveIt 参数(demo_hardware.launch.py 已处理,自组 launch 时勿只传 robot_description) |
| colcon 不识别包/插件发现不了 | `package.xml` 必须含 `<export><build_type>ament_cmake</build_type>`;改 xacro 后必须重新 `colcon build` 才生效(install/share 下的 xacro 是构建时拷贝) |

## 许可与致谢

本工程基于宇树官方 X86_SDK 开发;SDK 及 `joint_calib.txt` 版权归宇树所有,本仓库不含 SDK 文件。
