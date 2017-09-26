cmake_minimum_required(VERSION 2.8.3)
project(ble_client_cpp)

add_definitions(-std=c++11)
add_definitions(`pkg-config --cflags glib-2.0`)

set(CATKIN_DEPS
    roscpp
    )

find_package(catkin REQUIRED ${CATKIN_DEPS})

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ble_client_cpp
    DEPENDS ${CATKIN_DEPS}
    gatt
    glib-2.0
    pthread
    )

include_directories(
    include/ble_client_cpp
    )

add_library(ble_client_cpp
    src/ble_client.cpp
    )

target_link_libraries(ble_client_cpp
    ${catkin_LIBRARIES}
    gatt
    glib-2.0
    pthread
    )
