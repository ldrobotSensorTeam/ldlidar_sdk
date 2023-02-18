cmake_minimum_required(VERSION 2.8.3)
project(ldlidar)

message(STATUS "operation system is ${CMAKE_SYSTEM}")
if(CMAKE_SYSTEM_NAME MATCHES "Linux")
	message(STATUS "current platform: Linux ")
elseif(CMAKE_SYSTEM_NAME MATCHES "Windows")
	message(STATUS "current platform: Windows")
elseif(CMAKE_SYSTEM_NAME MATCHES "FreeBSD")
	message(STATUS "current platform: FreeBSD")
else()
	message(STATUS "other platform: ${CMAKE_SYSTEM_NAME}")
endif (CMAKE_SYSTEM_NAME MATCHES "Linux")

if(WIN32)
	message(STATUS "Now is windows")
elseif(APPLE)
	message(STATUS "Now is Apple systens.")
elseif(UNIX)
	message(STATUS "Now is UNIX-like OS's.")
endif()
message(STATUS "###################################")


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
  ${CMAKE_CURRENT_SOURCE_DIR}/include/
)

file(GLOB  LDLIDAR_DRIVER_SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/src/ldlidar_driver/*.cpp)

add_executable(${PROJECT_NAME}
  ${CMAKE_CURRENT_SOURCE_DIR}/sample/linux/demo.cpp
)

add_library(ldlidar_driver_static STATIC 
  ${LDLIDAR_DRIVER_SOURCE} 
)

add_library(ldlidar_driver_shared SHARED 
  ${LDLIDAR_DRIVER_SOURCE} 
)

# rename library name
set_target_properties (ldlidar_driver_static PROPERTIES OUTPUT_NAME "ldlidar_driver")
set_target_properties (ldlidar_driver_shared PROPERTIES OUTPUT_NAME "ldlidar_driver")

# binary file link to library
target_link_libraries(${PROJECT_NAME} ldlidar_driver_static pthread)

###########
## Install ##
###########

INSTALL(TARGETS ldlidar_driver_static ldlidar_driver_shared
  ARCHIVE DESTINATION lib/ldlidar_driver
  LIBRARY DESTINATION share/ldlidar_driver
)

INSTALL(DIRECTORY ${PROJECT_SOURCE_DIR}/include
  DESTINATION include
)
