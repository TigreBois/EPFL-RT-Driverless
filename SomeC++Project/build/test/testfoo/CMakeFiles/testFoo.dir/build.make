# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/masouye/DriverlessRT/EPFL-RT-Driverless/SomeC++Project

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/masouye/DriverlessRT/EPFL-RT-Driverless/SomeC++Project/build

# Include any dependencies generated for this target.
include test/testfoo/CMakeFiles/testFoo.dir/depend.make

# Include the progress variables for this target.
include test/testfoo/CMakeFiles/testFoo.dir/progress.make

# Include the compile flags for this target's objects.
include test/testfoo/CMakeFiles/testFoo.dir/flags.make

test/testfoo/CMakeFiles/testFoo.dir/main.cpp.o: test/testfoo/CMakeFiles/testFoo.dir/flags.make
test/testfoo/CMakeFiles/testFoo.dir/main.cpp.o: ../test/testfoo/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/masouye/DriverlessRT/EPFL-RT-Driverless/SomeC++Project/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object test/testfoo/CMakeFiles/testFoo.dir/main.cpp.o"
	cd /home/masouye/DriverlessRT/EPFL-RT-Driverless/SomeC++Project/build/test/testfoo && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/testFoo.dir/main.cpp.o -c /home/masouye/DriverlessRT/EPFL-RT-Driverless/SomeC++Project/test/testfoo/main.cpp

test/testfoo/CMakeFiles/testFoo.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/testFoo.dir/main.cpp.i"
	cd /home/masouye/DriverlessRT/EPFL-RT-Driverless/SomeC++Project/build/test/testfoo && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/masouye/DriverlessRT/EPFL-RT-Driverless/SomeC++Project/test/testfoo/main.cpp > CMakeFiles/testFoo.dir/main.cpp.i

test/testfoo/CMakeFiles/testFoo.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/testFoo.dir/main.cpp.s"
	cd /home/masouye/DriverlessRT/EPFL-RT-Driverless/SomeC++Project/build/test/testfoo && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/masouye/DriverlessRT/EPFL-RT-Driverless/SomeC++Project/test/testfoo/main.cpp -o CMakeFiles/testFoo.dir/main.cpp.s

test/testfoo/CMakeFiles/testFoo.dir/main.cpp.o.requires:

.PHONY : test/testfoo/CMakeFiles/testFoo.dir/main.cpp.o.requires

test/testfoo/CMakeFiles/testFoo.dir/main.cpp.o.provides: test/testfoo/CMakeFiles/testFoo.dir/main.cpp.o.requires
	$(MAKE) -f test/testfoo/CMakeFiles/testFoo.dir/build.make test/testfoo/CMakeFiles/testFoo.dir/main.cpp.o.provides.build
.PHONY : test/testfoo/CMakeFiles/testFoo.dir/main.cpp.o.provides

test/testfoo/CMakeFiles/testFoo.dir/main.cpp.o.provides.build: test/testfoo/CMakeFiles/testFoo.dir/main.cpp.o


test/testfoo/CMakeFiles/testFoo.dir/example_add.cpp.o: test/testfoo/CMakeFiles/testFoo.dir/flags.make
test/testfoo/CMakeFiles/testFoo.dir/example_add.cpp.o: ../test/testfoo/example_add.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/masouye/DriverlessRT/EPFL-RT-Driverless/SomeC++Project/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object test/testfoo/CMakeFiles/testFoo.dir/example_add.cpp.o"
	cd /home/masouye/DriverlessRT/EPFL-RT-Driverless/SomeC++Project/build/test/testfoo && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/testFoo.dir/example_add.cpp.o -c /home/masouye/DriverlessRT/EPFL-RT-Driverless/SomeC++Project/test/testfoo/example_add.cpp

test/testfoo/CMakeFiles/testFoo.dir/example_add.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/testFoo.dir/example_add.cpp.i"
	cd /home/masouye/DriverlessRT/EPFL-RT-Driverless/SomeC++Project/build/test/testfoo && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/masouye/DriverlessRT/EPFL-RT-Driverless/SomeC++Project/test/testfoo/example_add.cpp > CMakeFiles/testFoo.dir/example_add.cpp.i

test/testfoo/CMakeFiles/testFoo.dir/example_add.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/testFoo.dir/example_add.cpp.s"
	cd /home/masouye/DriverlessRT/EPFL-RT-Driverless/SomeC++Project/build/test/testfoo && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/masouye/DriverlessRT/EPFL-RT-Driverless/SomeC++Project/test/testfoo/example_add.cpp -o CMakeFiles/testFoo.dir/example_add.cpp.s

test/testfoo/CMakeFiles/testFoo.dir/example_add.cpp.o.requires:

.PHONY : test/testfoo/CMakeFiles/testFoo.dir/example_add.cpp.o.requires

test/testfoo/CMakeFiles/testFoo.dir/example_add.cpp.o.provides: test/testfoo/CMakeFiles/testFoo.dir/example_add.cpp.o.requires
	$(MAKE) -f test/testfoo/CMakeFiles/testFoo.dir/build.make test/testfoo/CMakeFiles/testFoo.dir/example_add.cpp.o.provides.build
.PHONY : test/testfoo/CMakeFiles/testFoo.dir/example_add.cpp.o.provides

test/testfoo/CMakeFiles/testFoo.dir/example_add.cpp.o.provides.build: test/testfoo/CMakeFiles/testFoo.dir/example_add.cpp.o


test/testfoo/CMakeFiles/testFoo.dir/example_subtract.cpp.o: test/testfoo/CMakeFiles/testFoo.dir/flags.make
test/testfoo/CMakeFiles/testFoo.dir/example_subtract.cpp.o: ../test/testfoo/example_subtract.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/masouye/DriverlessRT/EPFL-RT-Driverless/SomeC++Project/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object test/testfoo/CMakeFiles/testFoo.dir/example_subtract.cpp.o"
	cd /home/masouye/DriverlessRT/EPFL-RT-Driverless/SomeC++Project/build/test/testfoo && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/testFoo.dir/example_subtract.cpp.o -c /home/masouye/DriverlessRT/EPFL-RT-Driverless/SomeC++Project/test/testfoo/example_subtract.cpp

test/testfoo/CMakeFiles/testFoo.dir/example_subtract.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/testFoo.dir/example_subtract.cpp.i"
	cd /home/masouye/DriverlessRT/EPFL-RT-Driverless/SomeC++Project/build/test/testfoo && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/masouye/DriverlessRT/EPFL-RT-Driverless/SomeC++Project/test/testfoo/example_subtract.cpp > CMakeFiles/testFoo.dir/example_subtract.cpp.i

test/testfoo/CMakeFiles/testFoo.dir/example_subtract.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/testFoo.dir/example_subtract.cpp.s"
	cd /home/masouye/DriverlessRT/EPFL-RT-Driverless/SomeC++Project/build/test/testfoo && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/masouye/DriverlessRT/EPFL-RT-Driverless/SomeC++Project/test/testfoo/example_subtract.cpp -o CMakeFiles/testFoo.dir/example_subtract.cpp.s

test/testfoo/CMakeFiles/testFoo.dir/example_subtract.cpp.o.requires:

.PHONY : test/testfoo/CMakeFiles/testFoo.dir/example_subtract.cpp.o.requires

test/testfoo/CMakeFiles/testFoo.dir/example_subtract.cpp.o.provides: test/testfoo/CMakeFiles/testFoo.dir/example_subtract.cpp.o.requires
	$(MAKE) -f test/testfoo/CMakeFiles/testFoo.dir/build.make test/testfoo/CMakeFiles/testFoo.dir/example_subtract.cpp.o.provides.build
.PHONY : test/testfoo/CMakeFiles/testFoo.dir/example_subtract.cpp.o.provides

test/testfoo/CMakeFiles/testFoo.dir/example_subtract.cpp.o.provides.build: test/testfoo/CMakeFiles/testFoo.dir/example_subtract.cpp.o


# Object files for target testFoo
testFoo_OBJECTS = \
"CMakeFiles/testFoo.dir/main.cpp.o" \
"CMakeFiles/testFoo.dir/example_add.cpp.o" \
"CMakeFiles/testFoo.dir/example_subtract.cpp.o"

# External object files for target testFoo
testFoo_EXTERNAL_OBJECTS =

bin/testFoo: test/testfoo/CMakeFiles/testFoo.dir/main.cpp.o
bin/testFoo: test/testfoo/CMakeFiles/testFoo.dir/example_add.cpp.o
bin/testFoo: test/testfoo/CMakeFiles/testFoo.dir/example_subtract.cpp.o
bin/testFoo: test/testfoo/CMakeFiles/testFoo.dir/build.make
bin/testFoo: lib/libgtest.a
bin/testFoo: lib/libgtest_main.a
bin/testFoo: lib/libgmock.a
bin/testFoo: lib/libgmock_main.a
bin/testFoo: lib/libgtest_main.a
bin/testFoo: lib/libexample.a
bin/testFoo: lib/libgtest.a
bin/testFoo: test/testfoo/CMakeFiles/testFoo.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/masouye/DriverlessRT/EPFL-RT-Driverless/SomeC++Project/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable ../../bin/testFoo"
	cd /home/masouye/DriverlessRT/EPFL-RT-Driverless/SomeC++Project/build/test/testfoo && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/testFoo.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/testfoo/CMakeFiles/testFoo.dir/build: bin/testFoo

.PHONY : test/testfoo/CMakeFiles/testFoo.dir/build

test/testfoo/CMakeFiles/testFoo.dir/requires: test/testfoo/CMakeFiles/testFoo.dir/main.cpp.o.requires
test/testfoo/CMakeFiles/testFoo.dir/requires: test/testfoo/CMakeFiles/testFoo.dir/example_add.cpp.o.requires
test/testfoo/CMakeFiles/testFoo.dir/requires: test/testfoo/CMakeFiles/testFoo.dir/example_subtract.cpp.o.requires

.PHONY : test/testfoo/CMakeFiles/testFoo.dir/requires

test/testfoo/CMakeFiles/testFoo.dir/clean:
	cd /home/masouye/DriverlessRT/EPFL-RT-Driverless/SomeC++Project/build/test/testfoo && $(CMAKE_COMMAND) -P CMakeFiles/testFoo.dir/cmake_clean.cmake
.PHONY : test/testfoo/CMakeFiles/testFoo.dir/clean

test/testfoo/CMakeFiles/testFoo.dir/depend:
	cd /home/masouye/DriverlessRT/EPFL-RT-Driverless/SomeC++Project/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/masouye/DriverlessRT/EPFL-RT-Driverless/SomeC++Project /home/masouye/DriverlessRT/EPFL-RT-Driverless/SomeC++Project/test/testfoo /home/masouye/DriverlessRT/EPFL-RT-Driverless/SomeC++Project/build /home/masouye/DriverlessRT/EPFL-RT-Driverless/SomeC++Project/build/test/testfoo /home/masouye/DriverlessRT/EPFL-RT-Driverless/SomeC++Project/build/test/testfoo/CMakeFiles/testFoo.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/testfoo/CMakeFiles/testFoo.dir/depend

