cmake_minimum_required(VERSION 2.8.3)
project(ldlidar)

if(${CMAKE_BUILD_TYPE} MATCHES  "Release")
  set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -std=c++11 -Wall")
  message(STATUS "Mode: Release")
  message(STATUS "optional:-std=c++11 -Wall")
elseif(${CMAKE_BUILD_TYPE} MATCHES "Debug")
  set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -std=c++11 -Wall -Wextra -g2 -ggdb")
  message(STATUS "Mode: Debug")
  message(STATUS "optional:-std=c++11 -Wall -Wextra -g2 -ggdb")
else()
  set(CMAKE_BUILD_TYPE "Debug") 
  set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -std=c++11 -Wall -Wextra -g2 -ggdb")
  message(STATUS "Mode: Debug")
  message(STATUS "optional:-std=c++11 -Wall -Wextra -g2 -ggdb")
endif()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/../../include/
)

set(LDLIDAR_DRIVER_SOURCE_LINUX
  ${CMAKE_CURRENT_SOURCE_DIR}/../../src/ldlidar_driver.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../src/ldlidar_driver_linux.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../src/ldlidar_dataprocess.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../src/ldlidar_protocol.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../src/log_module.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../src/network_socket_interface_linux.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../src/serial_interface_linux.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../src/sl_transform.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../src/slbf.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../src/tofbf.cpp
)

add_executable(${PROJECT_NAME}
  demo.cpp
)

add_library(ldlidar_driver_static STATIC 
  ${LDLIDAR_DRIVER_SOURCE_LINUX} 
)

add_library(ldlidar_driver_shared SHARED 
  ${LDLIDAR_DRIVER_SOURCE_LINUX} 
)

# rename library name
set_target_properties (ldlidar_driver_static PROPERTIES OUTPUT_NAME "ldlidar_driver")
set_target_properties (ldlidar_driver_shared PROPERTIES OUTPUT_NAME "ldlidar_driver")

# binary file link to library
target_link_libraries(${PROJECT_NAME} ldlidar_driver_static pthread)

###########
## Install ##
###########

install(TARGETS ldlidar_driver_static ldlidar_driver_shared
  ARCHIVE DESTINATION lib/ldlidar_driver
  LIBRARY DESTINATION share/ldlidar_driver
)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include
  DESTINATION include
)
