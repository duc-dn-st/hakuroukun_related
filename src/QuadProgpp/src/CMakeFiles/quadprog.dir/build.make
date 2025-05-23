# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /root/catkin_ws/src/QuadProgpp

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /root/catkin_ws/src/QuadProgpp

# Include any dependencies generated for this target.
include src/CMakeFiles/quadprog.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/quadprog.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/quadprog.dir/flags.make

src/CMakeFiles/quadprog.dir/Array.cc.o: src/CMakeFiles/quadprog.dir/flags.make
src/CMakeFiles/quadprog.dir/Array.cc.o: src/Array.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/root/catkin_ws/src/QuadProgpp/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/CMakeFiles/quadprog.dir/Array.cc.o"
	cd /root/catkin_ws/src/QuadProgpp/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/quadprog.dir/Array.cc.o -c /root/catkin_ws/src/QuadProgpp/src/Array.cc

src/CMakeFiles/quadprog.dir/Array.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/quadprog.dir/Array.cc.i"
	cd /root/catkin_ws/src/QuadProgpp/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /root/catkin_ws/src/QuadProgpp/src/Array.cc > CMakeFiles/quadprog.dir/Array.cc.i

src/CMakeFiles/quadprog.dir/Array.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/quadprog.dir/Array.cc.s"
	cd /root/catkin_ws/src/QuadProgpp/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /root/catkin_ws/src/QuadProgpp/src/Array.cc -o CMakeFiles/quadprog.dir/Array.cc.s

src/CMakeFiles/quadprog.dir/QuadProg++.cc.o: src/CMakeFiles/quadprog.dir/flags.make
src/CMakeFiles/quadprog.dir/QuadProg++.cc.o: src/QuadProg++.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/root/catkin_ws/src/QuadProgpp/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/CMakeFiles/quadprog.dir/QuadProg++.cc.o"
	cd /root/catkin_ws/src/QuadProgpp/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/quadprog.dir/QuadProg++.cc.o -c /root/catkin_ws/src/QuadProgpp/src/QuadProg++.cc

src/CMakeFiles/quadprog.dir/QuadProg++.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/quadprog.dir/QuadProg++.cc.i"
	cd /root/catkin_ws/src/QuadProgpp/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /root/catkin_ws/src/QuadProgpp/src/QuadProg++.cc > CMakeFiles/quadprog.dir/QuadProg++.cc.i

src/CMakeFiles/quadprog.dir/QuadProg++.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/quadprog.dir/QuadProg++.cc.s"
	cd /root/catkin_ws/src/QuadProgpp/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /root/catkin_ws/src/QuadProgpp/src/QuadProg++.cc -o CMakeFiles/quadprog.dir/QuadProg++.cc.s

# Object files for target quadprog
quadprog_OBJECTS = \
"CMakeFiles/quadprog.dir/Array.cc.o" \
"CMakeFiles/quadprog.dir/QuadProg++.cc.o"

# External object files for target quadprog
quadprog_EXTERNAL_OBJECTS =

src/libquadprog.a: src/CMakeFiles/quadprog.dir/Array.cc.o
src/libquadprog.a: src/CMakeFiles/quadprog.dir/QuadProg++.cc.o
src/libquadprog.a: src/CMakeFiles/quadprog.dir/build.make
src/libquadprog.a: src/CMakeFiles/quadprog.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/root/catkin_ws/src/QuadProgpp/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX static library libquadprog.a"
	cd /root/catkin_ws/src/QuadProgpp/src && $(CMAKE_COMMAND) -P CMakeFiles/quadprog.dir/cmake_clean_target.cmake
	cd /root/catkin_ws/src/QuadProgpp/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/quadprog.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/quadprog.dir/build: src/libquadprog.a

.PHONY : src/CMakeFiles/quadprog.dir/build

src/CMakeFiles/quadprog.dir/clean:
	cd /root/catkin_ws/src/QuadProgpp/src && $(CMAKE_COMMAND) -P CMakeFiles/quadprog.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/quadprog.dir/clean

src/CMakeFiles/quadprog.dir/depend:
	cd /root/catkin_ws/src/QuadProgpp && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /root/catkin_ws/src/QuadProgpp /root/catkin_ws/src/QuadProgpp/src /root/catkin_ws/src/QuadProgpp /root/catkin_ws/src/QuadProgpp/src /root/catkin_ws/src/QuadProgpp/src/CMakeFiles/quadprog.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/quadprog.dir/depend

