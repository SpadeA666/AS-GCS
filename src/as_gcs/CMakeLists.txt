cmake_minimum_required(VERSION 3.0.2)
project(as_gcs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  sensor_msgs
  actionlib_msgs
)

add_message_files(FILES
  DeviceStatus.msg
  GcsHeartbeat.msg
  Waypoint.msg
)

add_service_files(FILES
  Takeoff.srv
  CancelTakeoff.srv
  Land.srv
  FlyUp.srv
  FlyDown.srv
  GoToPx4.srv
  SetPlanner.srv
  SetNavMode.srv
  StartFollow.srv
  StopFollow.srv
  AlignTarget.srv
  SetActuator.srv
  SetGeofence.srv
  EmergencyStop.srv
  CameraControl.srv
  YoloControl.srv
)

add_action_files(FILES
  Navigate.action
)

generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs sensor_msgs actionlib_msgs
)
