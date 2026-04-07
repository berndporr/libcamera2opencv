cmake_minimum_required(VERSION 3.25)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()

project(MetaDataPrinter
	DESCRIPTION "Prints the metadata of every frame on the screen"
	LANGUAGES CXX)

set (CMAKE_CXX_STANDARD 17)

set (CMAKE_CXX_FLAGS "-Wall -Winvalid-pch -Wnon-virtual-dtor -Wextra -Wno-unused-parameter")

include(GNUInstallDirs)

add_executable(printdata
  printdata.cpp
)

target_include_directories(printdata PUBLIC ${CMAKE_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/..)

target_link_libraries(printdata cam2opencv)
