# sub project
message(STATUS "operation system is ${CMAKE_SYSTEM}")

if(CMAKE_SYSTEM_NAME MATCHES "Linux")
message(STATUS "current platform: Linux ")

set(LDLIDAR_DRIVER_SOURCE_LINUX
  ${CMAKE_CURRENT_SOURCE_DIR}/src/ldlidar_driver.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/ldlidar_driver_linux.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/ldlidar_dataprocess.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/ldlidar_protocol.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/log_module.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/network_socket_interface_linux.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/serial_interface_linux.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/sl_transform.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/slbf.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/tofbf.cpp
)

add_library(ldlidar_driver STATIC 
  ${LDLIDAR_DRIVER_SOURCE_LINUX} 
)

elseif(CMAKE_SYSTEM_NAME MATCHES "Windows")
message(STATUS "current platform: Windows")

set(LDLIDAR_DRIVER_SOURCE_WIN
  ${CMAKE_CURRENT_SOURCE_DIR}/src/ldlidar_driver.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/ldlidar_driver_win.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/ldlidar_dataprocess.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/ldlidar_protocol.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/log_module.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/serial_interface_win.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/sl_transform.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/slbf.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/tofbf.cpp
)

add_library(ldlidar_driver STATIC 
  ${LDLIDAR_DRIVER_SOURCE_WIN} 
)

else()
	message(STATUS "other platform: ${CMAKE_SYSTEM_NAME}")
endif (CMAKE_SYSTEM_NAME MATCHES "Linux")


