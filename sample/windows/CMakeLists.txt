cmake_minimum_required(VERSION 3.5)

project (ldlidar_driver_win LANGUAGES CXX)

include_directories(
  ${PROJECT_SOURCE_DIR}/
  ${PROJECT_SOURCE_DIR}/../../include
)

add_executable(demo 
  ${PROJECT_SOURCE_DIR}/windemo.cpp 
  ${PROJECT_SOURCE_DIR}/ldlidar_driver_win.cpp
  ${PROJECT_SOURCE_DIR}/commport.cpp
  ${PROJECT_SOURCE_DIR}/../../src/ldlidar_driver/lipkg.cpp
  ${PROJECT_SOURCE_DIR}/../../src/ldlidar_driver/log_module.cpp
  ${PROJECT_SOURCE_DIR}/../../src/ldlidar_driver/sl_transform.cpp
  ${PROJECT_SOURCE_DIR}/../../src/ldlidar_driver/slbf.cpp
  ${PROJECT_SOURCE_DIR}/../../src/ldlidar_driver/tofbf.cpp
)
target_compile_features(demo PUBLIC cxx_std_11)