cmake_minimum_required(VERSION 3.8)
project(zfy_arm_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JointGoal.msg"
  "msg/MotorRegistryEntry.msg"
  "msg/MotorMitCommand.msg"
  "srv/SetMotorMode.srv"
  "srv/ReadMotorParam.srv"
  "srv/WriteMotorParam.srv"
  "srv/SaveMotorParam.srv"
  "srv/GetMotorStatus.srv"
  "srv/ConfigureMotors.srv"
  "srv/SetMotorEnable.srv"
  "srv/SetMotorSending.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
