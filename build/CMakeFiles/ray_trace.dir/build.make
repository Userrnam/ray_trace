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
CMAKE_COMMAND = /opt/local/bin/cmake

# The command to remove a file.
RM = /opt/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/antonkondratuk/Desktop/cpp/ray_trace

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/antonkondratuk/Desktop/cpp/ray_trace/build

# Include any dependencies generated for this target.
include CMakeFiles/ray_trace.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/ray_trace.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/ray_trace.dir/flags.make

CMakeFiles/ray_trace.dir/src/stb_impl.cpp.o: CMakeFiles/ray_trace.dir/flags.make
CMakeFiles/ray_trace.dir/src/stb_impl.cpp.o: ../src/stb_impl.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/antonkondratuk/Desktop/cpp/ray_trace/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/ray_trace.dir/src/stb_impl.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ray_trace.dir/src/stb_impl.cpp.o -c /Users/antonkondratuk/Desktop/cpp/ray_trace/src/stb_impl.cpp

CMakeFiles/ray_trace.dir/src/stb_impl.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ray_trace.dir/src/stb_impl.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/antonkondratuk/Desktop/cpp/ray_trace/src/stb_impl.cpp > CMakeFiles/ray_trace.dir/src/stb_impl.cpp.i

CMakeFiles/ray_trace.dir/src/stb_impl.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ray_trace.dir/src/stb_impl.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/antonkondratuk/Desktop/cpp/ray_trace/src/stb_impl.cpp -o CMakeFiles/ray_trace.dir/src/stb_impl.cpp.s

CMakeFiles/ray_trace.dir/src/main.cpp.o: CMakeFiles/ray_trace.dir/flags.make
CMakeFiles/ray_trace.dir/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/antonkondratuk/Desktop/cpp/ray_trace/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/ray_trace.dir/src/main.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ray_trace.dir/src/main.cpp.o -c /Users/antonkondratuk/Desktop/cpp/ray_trace/src/main.cpp

CMakeFiles/ray_trace.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ray_trace.dir/src/main.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/antonkondratuk/Desktop/cpp/ray_trace/src/main.cpp > CMakeFiles/ray_trace.dir/src/main.cpp.i

CMakeFiles/ray_trace.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ray_trace.dir/src/main.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/antonkondratuk/Desktop/cpp/ray_trace/src/main.cpp -o CMakeFiles/ray_trace.dir/src/main.cpp.s

# Object files for target ray_trace
ray_trace_OBJECTS = \
"CMakeFiles/ray_trace.dir/src/stb_impl.cpp.o" \
"CMakeFiles/ray_trace.dir/src/main.cpp.o"

# External object files for target ray_trace
ray_trace_EXTERNAL_OBJECTS =

ray_trace: CMakeFiles/ray_trace.dir/src/stb_impl.cpp.o
ray_trace: CMakeFiles/ray_trace.dir/src/main.cpp.o
ray_trace: CMakeFiles/ray_trace.dir/build.make
ray_trace: CMakeFiles/ray_trace.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/antonkondratuk/Desktop/cpp/ray_trace/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable ray_trace"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ray_trace.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/ray_trace.dir/build: ray_trace

.PHONY : CMakeFiles/ray_trace.dir/build

CMakeFiles/ray_trace.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ray_trace.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ray_trace.dir/clean

CMakeFiles/ray_trace.dir/depend:
	cd /Users/antonkondratuk/Desktop/cpp/ray_trace/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/antonkondratuk/Desktop/cpp/ray_trace /Users/antonkondratuk/Desktop/cpp/ray_trace /Users/antonkondratuk/Desktop/cpp/ray_trace/build /Users/antonkondratuk/Desktop/cpp/ray_trace/build /Users/antonkondratuk/Desktop/cpp/ray_trace/build/CMakeFiles/ray_trace.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ray_trace.dir/depend

