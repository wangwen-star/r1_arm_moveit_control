cmake_minimum_required(VERSION 3.8)
project(r1_arm_hardware)


# 注意:CATKIN_INSTALL_INTO_PREFIX_ROOT 对 ament_cmake 无效(ament 不读取该变量),
# 已删除。colcon 识别构建类型看的是 package.xml 的 <export><build_type> 标签


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_srvs REQUIRED)

set(R1_SDK_ROOT "/home/wangwenyu/桌面/X86_SDK/r1_arm")

# link_directories 放到add_library前面！
link_directories(${R1_SDK_ROOT}/thirdparty/motor_controller/lib)

# arm_motor_bus.cpp 必须一起编译:SDK 只提供源码(不发布 lib),
# 只链接 libmotor_controller.so 会导致 ArmMotorBus 符号未定义
add_library(r1_arm_hardware SHARED
  src/r1_arm_hardware.cpp
  ${R1_SDK_ROOT}/src/arm_motor_bus.cpp
)

# SDK 根路径编译宏(on_configure 中 calib_path 相对路径兜底用)
target_compile_definitions(r1_arm_hardware PRIVATE
  R1_SDK_ROOT_PATH="${R1_SDK_ROOT}"
)

# PRIVATE，不要PUBLIC，SDK头文件不需要对外导出
target_include_directories(r1_arm_hardware PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${R1_SDK_ROOT}/include
  ${R1_SDK_ROOT}/thirdparty/motor_controller/include
)

target_link_libraries(r1_arm_hardware
  motor_controller
)

ament_target_dependencies(r1_arm_hardware
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_srvs
)

pluginlib_export_plugin_description_file(hardware_interface r1_arm_hardware.xml)

install(TARGETS
  r1_arm_hardware
  DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()