# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/xiao/workspace/msl/CAMBADA/rtdb_new

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/xiao/workspace/msl/CAMBADA/rtdb_new/build

# Include any dependencies generated for this target.
include example/CMakeFiles/write.dir/depend.make

# Include the progress variables for this target.
include example/CMakeFiles/write.dir/progress.make

# Include the compile flags for this target's objects.
include example/CMakeFiles/write.dir/flags.make

example/CMakeFiles/write.dir/write.c.o: example/CMakeFiles/write.dir/flags.make
example/CMakeFiles/write.dir/write.c.o: ../example/write.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/xiao/workspace/msl/CAMBADA/rtdb_new/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object example/CMakeFiles/write.dir/write.c.o"
	cd /home/xiao/workspace/msl/CAMBADA/rtdb_new/build/example && /usr/bin/gcc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/write.dir/write.c.o   -c /home/xiao/workspace/msl/CAMBADA/rtdb_new/example/write.c

example/CMakeFiles/write.dir/write.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/write.dir/write.c.i"
	cd /home/xiao/workspace/msl/CAMBADA/rtdb_new/build/example && /usr/bin/gcc  $(C_DEFINES) $(C_FLAGS) -E /home/xiao/workspace/msl/CAMBADA/rtdb_new/example/write.c > CMakeFiles/write.dir/write.c.i

example/CMakeFiles/write.dir/write.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/write.dir/write.c.s"
	cd /home/xiao/workspace/msl/CAMBADA/rtdb_new/build/example && /usr/bin/gcc  $(C_DEFINES) $(C_FLAGS) -S /home/xiao/workspace/msl/CAMBADA/rtdb_new/example/write.c -o CMakeFiles/write.dir/write.c.s

example/CMakeFiles/write.dir/write.c.o.requires:
.PHONY : example/CMakeFiles/write.dir/write.c.o.requires

example/CMakeFiles/write.dir/write.c.o.provides: example/CMakeFiles/write.dir/write.c.o.requires
	$(MAKE) -f example/CMakeFiles/write.dir/build.make example/CMakeFiles/write.dir/write.c.o.provides.build
.PHONY : example/CMakeFiles/write.dir/write.c.o.provides

example/CMakeFiles/write.dir/write.c.o.provides.build: example/CMakeFiles/write.dir/write.c.o

# Object files for target write
write_OBJECTS = \
"CMakeFiles/write.dir/write.c.o"

# External object files for target write
write_EXTERNAL_OBJECTS =

bin/write: example/CMakeFiles/write.dir/write.c.o
bin/write: lib/librtdb.a
bin/write: example/CMakeFiles/write.dir/build.make
bin/write: example/CMakeFiles/write.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking C executable ../bin/write"
	cd /home/xiao/workspace/msl/CAMBADA/rtdb_new/build/example && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/write.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
example/CMakeFiles/write.dir/build: bin/write
.PHONY : example/CMakeFiles/write.dir/build

example/CMakeFiles/write.dir/requires: example/CMakeFiles/write.dir/write.c.o.requires
.PHONY : example/CMakeFiles/write.dir/requires

example/CMakeFiles/write.dir/clean:
	cd /home/xiao/workspace/msl/CAMBADA/rtdb_new/build/example && $(CMAKE_COMMAND) -P CMakeFiles/write.dir/cmake_clean.cmake
.PHONY : example/CMakeFiles/write.dir/clean

example/CMakeFiles/write.dir/depend:
	cd /home/xiao/workspace/msl/CAMBADA/rtdb_new/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/xiao/workspace/msl/CAMBADA/rtdb_new /home/xiao/workspace/msl/CAMBADA/rtdb_new/example /home/xiao/workspace/msl/CAMBADA/rtdb_new/build /home/xiao/workspace/msl/CAMBADA/rtdb_new/build/example /home/xiao/workspace/msl/CAMBADA/rtdb_new/build/example/CMakeFiles/write.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : example/CMakeFiles/write.dir/depend

