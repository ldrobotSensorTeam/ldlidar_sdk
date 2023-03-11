cmake_minimum_required(VERSION 3.5)

project (ldlidar_driver_win LANGUAGES CXX)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/../../include
)

add_executable(demo 
  ${CMAKE_CURRENT_SOURCE_DIR}/windemo.cpp 
  ${CMAKE_CURRENT_SOURCE_DIR}/../../src/ldlidar_driver.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../src/ldlidar_driver_win.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../src/serial_interface_win.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../src/ldlidar_dataprocess.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../src/ldlidar_protocol.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../src/log_module.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../src/sl_transform.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../src/slbf.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../src/tofbf.cpp
)
target_compile_features(demo PUBLIC cxx_std_11)