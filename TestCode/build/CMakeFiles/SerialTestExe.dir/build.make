# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.15

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/cooper/SerialCommsTesting/TestCode

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/cooper/SerialCommsTesting/TestCode/build

# Include any dependencies generated for this target.
include CMakeFiles/SerialTestExe.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/SerialTestExe.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/SerialTestExe.dir/flags.make

CMakeFiles/SerialTestExe.dir/test.cpp.o: CMakeFiles/SerialTestExe.dir/flags.make
CMakeFiles/SerialTestExe.dir/test.cpp.o: ../test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cooper/SerialCommsTesting/TestCode/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/SerialTestExe.dir/test.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/SerialTestExe.dir/test.cpp.o -c /home/cooper/SerialCommsTesting/TestCode/test.cpp

CMakeFiles/SerialTestExe.dir/test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/SerialTestExe.dir/test.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cooper/SerialCommsTesting/TestCode/test.cpp > CMakeFiles/SerialTestExe.dir/test.cpp.i

CMakeFiles/SerialTestExe.dir/test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/SerialTestExe.dir/test.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cooper/SerialCommsTesting/TestCode/test.cpp -o CMakeFiles/SerialTestExe.dir/test.cpp.s

# Object files for target SerialTestExe
SerialTestExe_OBJECTS = \
"CMakeFiles/SerialTestExe.dir/test.cpp.o"

# External object files for target SerialTestExe
SerialTestExe_EXTERNAL_OBJECTS =

SerialTestExe: CMakeFiles/SerialTestExe.dir/test.cpp.o
SerialTestExe: CMakeFiles/SerialTestExe.dir/build.make
SerialTestExe: CMakeFiles/SerialTestExe.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/cooper/SerialCommsTesting/TestCode/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable SerialTestExe"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/SerialTestExe.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/SerialTestExe.dir/build: SerialTestExe

.PHONY : CMakeFiles/SerialTestExe.dir/build

CMakeFiles/SerialTestExe.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/SerialTestExe.dir/cmake_clean.cmake
.PHONY : CMakeFiles/SerialTestExe.dir/clean

CMakeFiles/SerialTestExe.dir/depend:
	cd /home/cooper/SerialCommsTesting/TestCode/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cooper/SerialCommsTesting/TestCode /home/cooper/SerialCommsTesting/TestCode /home/cooper/SerialCommsTesting/TestCode/build /home/cooper/SerialCommsTesting/TestCode/build /home/cooper/SerialCommsTesting/TestCode/build/CMakeFiles/SerialTestExe.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/SerialTestExe.dir/depend
