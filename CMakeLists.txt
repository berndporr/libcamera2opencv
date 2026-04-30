cmake_minimum_required(VERSION 3.25)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()

project(LibcameraToOpenCVWrapper
	DESCRIPTION "Makes life easier for people needing to use libcamera"
	LANGUAGES CXX)

set (CMAKE_CXX_STANDARD 17)

set (CMAKE_CXX_FLAGS "-pedantic -Wall -Winvalid-pch -Wnon-virtual-dtor -Wextra -Wno-unused-parameter")

set (CMAKE_EXPORT_COMPILE_COMMANDS 1)

find_package(PkgConfig)
find_package(OpenCV REQUIRED)
pkg_check_modules(TURBOJPEG REQUIRED libturbojpeg)
pkg_check_modules(LIBCAMERA REQUIRED IMPORTED_TARGET libcamera)

message(STATUS "libcamera library found:")
message(STATUS "    version: ${LIBCAMERA_VERSION}")
message(STATUS "    libraries: ${LIBCAMERA_LINK_LIBRARIES}")
message(STATUS "    include path: ${LIBCAMERA_INCLUDE_DIRS}")

message(STATUS "opencv:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

message(STATUS "turbojpeg:")
message(STATUS "    version: ${TURBOJPEG_VERSION}")
message(STATUS "    libraries: ${TURBOJPEG_LINK_LIBRARIES}")
message(STATUS "    include path: ${TURBOJPEG_INCLUDE_DIRS}")

include(GNUInstallDirs)

add_subdirectory(metadataprinter)

add_library(cam2opencv STATIC libcam2opencv.cpp libcam2opencv_format_converter.cpp)

target_include_directories(cam2opencv PUBLIC
	${CMAKE_SOURCE_DIR}
	${LIBCAMERA_INCLUDE_DIRS}
	${OPENCV_INCLUDE_DIRS}
	${TURBOJPEG_INCLUDE_DIRS}
	)

target_link_libraries(cam2opencv PUBLIC PkgConfig::LIBCAMERA ${OpenCV_LIBS} ${TURBOJPEG_LINK_LIBRARIES})

set_target_properties(cam2opencv PROPERTIES
  PUBLIC_HEADER libcam2opencv.h
  PRIVATE_HEADER libcam2opencv_format_converter.h)

install(TARGETS cam2opencv EXPORT cam2opencv-targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  PRIVATE_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
