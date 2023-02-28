cmake_minimum_required(VERSION 3.5)

project (ldlidar_driver_win LANGUAGES CXX)

include_directories(
  ${PROJECT_SOURCE_DIR}/../../include
)

add_executable(demo 
  ${PROJECT_SOURCE_DIR}/windemo.cpp 
  ${PROJECT_SOURCE_DIR}/../../src/ldlidar_driver.cpp
  ${PROJECT_SOURCE_DIR}/../../src/ldlidar_driver_win.cpp
  ${PROJECT_SOURCE_DIR}/../../src/serial_interface_win.cpp
  ${PROJECT_SOURCE_DIR}/../../src/lipkg.cpp
  ${PROJECT_SOURCE_DIR}/../../src/log_module.cpp
  ${PROJECT_SOURCE_DIR}/../../src/sl_transform.cpp
  ${PROJECT_SOURCE_DIR}/../../src/slbf.cpp
  ${PROJECT_SOURCE_DIR}/../../src/tofbf.cpp
)
target_compile_features(demo PUBLIC cxx_std_11)