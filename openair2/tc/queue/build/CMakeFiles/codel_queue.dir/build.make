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
CMAKE_SOURCE_DIR = /home/mir/workspace/tc/queue

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/mir/workspace/tc/queue/build

# Include any dependencies generated for this target.
include CMakeFiles/codel_queue.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/codel_queue.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/codel_queue.dir/flags.make

CMakeFiles/codel_queue.dir/codel/codel_queue.c.o: CMakeFiles/codel_queue.dir/flags.make
CMakeFiles/codel_queue.dir/codel/codel_queue.c.o: ../codel/codel_queue.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/mir/workspace/tc/queue/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object CMakeFiles/codel_queue.dir/codel/codel_queue.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/codel_queue.dir/codel/codel_queue.c.o   -c /home/mir/workspace/tc/queue/codel/codel_queue.c

CMakeFiles/codel_queue.dir/codel/codel_queue.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/codel_queue.dir/codel/codel_queue.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/mir/workspace/tc/queue/codel/codel_queue.c > CMakeFiles/codel_queue.dir/codel/codel_queue.c.i

CMakeFiles/codel_queue.dir/codel/codel_queue.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/codel_queue.dir/codel/codel_queue.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/mir/workspace/tc/queue/codel/codel_queue.c -o CMakeFiles/codel_queue.dir/codel/codel_queue.c.s

CMakeFiles/codel_queue.dir/home/mir/workspace/tc/alg_ds/alg/defer.c.o: CMakeFiles/codel_queue.dir/flags.make
CMakeFiles/codel_queue.dir/home/mir/workspace/tc/alg_ds/alg/defer.c.o: /home/mir/workspace/tc/alg_ds/alg/defer.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/mir/workspace/tc/queue/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object CMakeFiles/codel_queue.dir/home/mir/workspace/tc/alg_ds/alg/defer.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/codel_queue.dir/home/mir/workspace/tc/alg_ds/alg/defer.c.o   -c /home/mir/workspace/tc/alg_ds/alg/defer.c

CMakeFiles/codel_queue.dir/home/mir/workspace/tc/alg_ds/alg/defer.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/codel_queue.dir/home/mir/workspace/tc/alg_ds/alg/defer.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/mir/workspace/tc/alg_ds/alg/defer.c > CMakeFiles/codel_queue.dir/home/mir/workspace/tc/alg_ds/alg/defer.c.i

CMakeFiles/codel_queue.dir/home/mir/workspace/tc/alg_ds/alg/defer.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/codel_queue.dir/home/mir/workspace/tc/alg_ds/alg/defer.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/mir/workspace/tc/alg_ds/alg/defer.c -o CMakeFiles/codel_queue.dir/home/mir/workspace/tc/alg_ds/alg/defer.c.s

# Object files for target codel_queue
codel_queue_OBJECTS = \
"CMakeFiles/codel_queue.dir/codel/codel_queue.c.o" \
"CMakeFiles/codel_queue.dir/home/mir/workspace/tc/alg_ds/alg/defer.c.o"

# External object files for target codel_queue
codel_queue_EXTERNAL_OBJECTS =

libcodel_queue.so: CMakeFiles/codel_queue.dir/codel/codel_queue.c.o
libcodel_queue.so: CMakeFiles/codel_queue.dir/home/mir/workspace/tc/alg_ds/alg/defer.c.o
libcodel_queue.so: CMakeFiles/codel_queue.dir/build.make
libcodel_queue.so: CMakeFiles/codel_queue.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/mir/workspace/tc/queue/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking C shared library libcodel_queue.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/codel_queue.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/codel_queue.dir/build: libcodel_queue.so

.PHONY : CMakeFiles/codel_queue.dir/build

CMakeFiles/codel_queue.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/codel_queue.dir/cmake_clean.cmake
.PHONY : CMakeFiles/codel_queue.dir/clean

CMakeFiles/codel_queue.dir/depend:
	cd /home/mir/workspace/tc/queue/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/mir/workspace/tc/queue /home/mir/workspace/tc/queue /home/mir/workspace/tc/queue/build /home/mir/workspace/tc/queue/build /home/mir/workspace/tc/queue/build/CMakeFiles/codel_queue.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/codel_queue.dir/depend

