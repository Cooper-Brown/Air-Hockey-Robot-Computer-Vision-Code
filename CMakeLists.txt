cmake_minimum_required (VERSION 3.0)

project ("FYP-Logic-CPP")

set (MAIN_SOURCE "./main.cpp")
set (COORDINATE_CLASS_SOURCE "./Coordinate.cpp")
set (LINE_CLASSS_SOURCE "./Line.cpp")
set (VECTOR_CLASS_SOURCE "./Vector.cpp")
set (CORNER_CLASS_SOURCE "./Corner.cpp")
set (PUCK_CLASS_SOURCE "./Puck.cpp")
set (AIR_HOCKEY_TABLE_CLASS_SOURCE "./AirHockeyTable.cpp")
set (MAT_DRAW_FUNCTIONS_CLASS_SOURCE "./MatDrawFunctions.cpp")

set (PROJ_BIN_LABEL "fyp-logic-exe")

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# Include the directories containing libraries
include_directories ("${CMAKE_CURRENT_SOURCE_DIR}/lib")
add_subdirectory (lib)

set (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH};${V4L2_HELPER_LIB_INSTALL_PATH}")

# Specify the compiler flags
set (GCC_COMPILE_FLAGS -Wall -Wpedantic -Wextra -O3 -Wshadow -std=c++11 -g)
add_compile_options (${GCC_COMPILE_FLAGS})

add_executable (
    ${PROJ_BIN_LABEL} 
    ${MAIN_SOURCE} 
    ${COORDINATE_CLASS_SOURCE} 
    ${LINE_CLASSS_SOURCE} 
    ${VECTOR_CLASS_SOURCE} 
    ${CORNER_CLASS_SOURCE} 
    ${PUCK_CLASS_SOURCE} 
    ${AIR_HOCKEY_TABLE_CLASS_SOURCE} 
    ${MAT_DRAW_FUNCTIONS_CLASS_SOURCE}
)
target_include_directories (${PROJ_BIN_LABEL} PUBLIC ${V4L2_HELPER_LIB_INCLUDE_DIR})
target_link_libraries (${PROJ_BIN_LABEL} v4l2_helper)
target_link_libraries (${PROJ_BIN_LABEL} ${OpenCV_LIBS})

install (
	TARGETS
	${PROJ_BIN_LABEL}
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
