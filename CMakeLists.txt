cmake_minimum_required (VERSION 3.0)

project ("OpenCV-Camera-Calibration")

set (MAIN_SOURCE "./main.cpp")

set (OPENCV_CALIB_BIN "opencv-calib-camera")

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# Include the directories containing libraries
include_directories ("${CMAKE_CURRENT_SOURCE_DIR}/lib")
add_subdirectory (lib)

set (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH};${V4L2_HELPER_LIB_INSTALL_PATH}")

# Specify the compiler flags
set (GCC_COMPILE_FLAGS -Wall -Wpedantic -Wextra -O3 -Wshadow -std=c++11 -g)
add_compile_options (${GCC_COMPILE_FLAGS})

add_executable (${OPENCV_CALIB_BIN} ${MAIN_SOURCE})
target_include_directories (${OPENCV_CALIB_BIN} PUBLIC ${V4L2_HELPER_LIB_INCLUDE_DIR})
target_link_libraries (${OPENCV_CALIB_BIN} v4l2_helper)
target_link_libraries (${OPENCV_CALIB_BIN} ${OpenCV_LIBS})

install (
	TARGETS
	${OPENCV_CALIB_BIN}
	RUNTIME DESTINATION bin
)

# uninstall target
# Ref: https://gitlab.kitware.com/cmake/community/wikis/FAQ#can-i-do-make-uninstall-with-cmake
if(NOT TARGET uninstall)
    configure_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
        "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
        IMMEDIATE @ONLY)

    add_custom_target(uninstall
        COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
endif()
