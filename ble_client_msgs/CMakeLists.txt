cmake_minimum_required(VERSION 2.8.3)
project(ble_client_msgs)

find_package(catkin REQUIRED COMPONENTS
    std_msgs
    message_generation)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  HandleByteArray.msg
  HandleUInt8Array.msg
  HandleUInt16Array.msg
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs message_runtime
)
