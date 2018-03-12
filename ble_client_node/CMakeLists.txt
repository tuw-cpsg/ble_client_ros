cmake_minimum_required(VERSION 2.8.3)
project(ble_client_node)

add_definitions(-std=c++11)
add_definitions(`pkg-config --cflags glib-2.0`)

set(CATKIN_DEPS
    roscpp
    ble_client_cpp
    ble_client_msgs
    )

find_package(catkin REQUIRED ${CATKIN_DEPS})

catkin_package(
    #  INCLUDE_DIRS include
    DEPENDS ${CATKIN_DEPS}
    glib-2.0
    pthread
    )

include_directories(
    ${catkin_INCLUDE_DIRS}
    )

add_executable(ble_client_node
    src/node.cpp
    )

target_link_libraries(ble_client_node ${catkin_LIBRARIES} glib-2.0 pthread)

## Mark executables and/or libraries for installation
install(TARGETS ble_client_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
