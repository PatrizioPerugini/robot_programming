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
CMAKE_SOURCE_DIR = /home/patrizio/Desktop/IK/02_mat_and_vec

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/patrizio/Desktop/IK/02_mat_and_vec

# Include any dependencies generated for this target.
include CMakeFiles/static_mat_test.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/static_mat_test.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/static_mat_test.dir/flags.make

CMakeFiles/static_mat_test.dir/static_mat_test.cpp.o: CMakeFiles/static_mat_test.dir/flags.make
CMakeFiles/static_mat_test.dir/static_mat_test.cpp.o: static_mat_test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/patrizio/Desktop/IK/02_mat_and_vec/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/static_mat_test.dir/static_mat_test.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/static_mat_test.dir/static_mat_test.cpp.o -c /home/patrizio/Desktop/IK/02_mat_and_vec/static_mat_test.cpp

CMakeFiles/static_mat_test.dir/static_mat_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/static_mat_test.dir/static_mat_test.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/patrizio/Desktop/IK/02_mat_and_vec/static_mat_test.cpp > CMakeFiles/static_mat_test.dir/static_mat_test.cpp.i

CMakeFiles/static_mat_test.dir/static_mat_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/static_mat_test.dir/static_mat_test.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/patrizio/Desktop/IK/02_mat_and_vec/static_mat_test.cpp -o CMakeFiles/static_mat_test.dir/static_mat_test.cpp.s

# Object files for target static_mat_test
static_mat_test_OBJECTS = \
"CMakeFiles/static_mat_test.dir/static_mat_test.cpp.o"

# External object files for target static_mat_test
static_mat_test_EXTERNAL_OBJECTS =

static_mat_test: CMakeFiles/static_mat_test.dir/static_mat_test.cpp.o
static_mat_test: CMakeFiles/static_mat_test.dir/build.make
static_mat_test: CMakeFiles/static_mat_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/patrizio/Desktop/IK/02_mat_and_vec/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable static_mat_test"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/static_mat_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/static_mat_test.dir/build: static_mat_test

.PHONY : CMakeFiles/static_mat_test.dir/build

CMakeFiles/static_mat_test.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/static_mat_test.dir/cmake_clean.cmake
.PHONY : CMakeFiles/static_mat_test.dir/clean

CMakeFiles/static_mat_test.dir/depend:
	cd /home/patrizio/Desktop/IK/02_mat_and_vec && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/patrizio/Desktop/IK/02_mat_and_vec /home/patrizio/Desktop/IK/02_mat_and_vec /home/patrizio/Desktop/IK/02_mat_and_vec /home/patrizio/Desktop/IK/02_mat_and_vec /home/patrizio/Desktop/IK/02_mat_and_vec/CMakeFiles/static_mat_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/static_mat_test.dir/depend

