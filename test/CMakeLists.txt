cmake_minimum_required(VERSION 2.8.3)
project(test)

if(${CMAKE_BUILD_TYPE} MATCHES  "Release")
  #set(CMAKE_BUILD_TYPE "Release") 
  set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -std=c++11 -Wall")
  #set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -std=c++11 -O3 -Wall")
  message(STATUS "Mode: Release")
  message(STATUS "optional:-std=c++11 -Wall")
else()
  #set(CMAKE_BUILD_TYPE "Debug") 
  set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -std=c++11 -Wall -Wextra -Wpedantic -g2 -ggdb")
  #set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -std=c++11 -O2 -Wall -Wextra -Wpedantic -g2 -ggdb")
  message(STATUS "Mode: Debug")
  message(STATUS "optional:-std=c++11 -Wall -Wextra -Wpedantic -g2 -ggdb")
endif()

if(${USE_WIRINGPI_LIB} MATCHES "ON")
  add_definitions(-D"USE_WIRINGPI")
  message(STATUS "-- #define USE_WIRINGPI")
endif()


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/../include/
)

set(LDLIDAR_DRIVER_SOURCE_LINUX
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/ldlidar_driver.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/ldlidar_driver_linux.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/ldlidar_protocol.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/ldlidar_dataprocess.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/log_module.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/network_socket_interface_linux.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/serial_interface_linux.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/sl_transform.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/slbf.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/tofbf.cpp
)

add_executable(${PROJECT_NAME}_node 
  ${CMAKE_CURRENT_SOURCE_DIR}/test.cpp
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
if(${USE_WIRINGPI_LIB} MATCHES "ON")
target_link_libraries(${PROJECT_NAME}_node 
  ldlidar_driver_static 
  pthread 
  wiringPi
)
else()
target_link_libraries(${PROJECT_NAME}_node 
  ldlidar_driver_static 
  pthread 
)
endif()

###########
## Install ##
###########

#INSTALL(TARGETS ldlidar_driver_static ldlidar_driver_shared
#  ARCHIVE DESTINATION lib
#  LIBRARY DESTINATION share
#)

#INSTALL(DIRECTORY ${PROJECT_SOURCE_DIR}/ldlidar_driver/include
#  DESTINATION include
#)
