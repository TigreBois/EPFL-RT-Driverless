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

# Utility rule file for coverage_testFoo.

# Include the progress variables for this target.
include test/testfoo/CMakeFiles/coverage_testFoo.dir/progress.make

test/testfoo/CMakeFiles/coverage_testFoo:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/masouye/DriverlessRT/EPFL-RT-Driverless/SomeC++Project/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Resetting code coverage counters to zero."
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Processing code coverage counters and generating report."
	/usr/bin/lcov --directory . --zerocounters
	/home/masouye/DriverlessRT/EPFL-RT-Driverless/SomeC++Project/build/bin/testFoo
	/usr/bin/lcov --directory . --capture --output-file coverage_testFoo_dir.info --rc lcov_branch_coverage=1
	/usr/bin/lcov --remove coverage_testFoo_dir.info 'build/*' 'tests/*' '/usr/*' --output-file coverage_testFoo_dir.info.cleaned
	/usr/bin/genhtml -o coverage_testFoo_dir coverage_testFoo_dir.info.cleaned
	/usr/bin/cmake -E remove coverage_testFoo_dir.info coverage_testFoo_dir.info.cleaned

coverage_testFoo: test/testfoo/CMakeFiles/coverage_testFoo
coverage_testFoo: test/testfoo/CMakeFiles/coverage_testFoo.dir/build.make
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Open ./coverage_testFoo_dir/index.html in your browser to view the coverage report."
.PHONY : coverage_testFoo

# Rule to build all files generated by this target.
test/testfoo/CMakeFiles/coverage_testFoo.dir/build: coverage_testFoo

.PHONY : test/testfoo/CMakeFiles/coverage_testFoo.dir/build

test/testfoo/CMakeFiles/coverage_testFoo.dir/clean:
	cd /home/masouye/DriverlessRT/EPFL-RT-Driverless/SomeC++Project/build/test/testfoo && $(CMAKE_COMMAND) -P CMakeFiles/coverage_testFoo.dir/cmake_clean.cmake
.PHONY : test/testfoo/CMakeFiles/coverage_testFoo.dir/clean

test/testfoo/CMakeFiles/coverage_testFoo.dir/depend:
	cd /home/masouye/DriverlessRT/EPFL-RT-Driverless/SomeC++Project/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/masouye/DriverlessRT/EPFL-RT-Driverless/SomeC++Project /home/masouye/DriverlessRT/EPFL-RT-Driverless/SomeC++Project/test/testfoo /home/masouye/DriverlessRT/EPFL-RT-Driverless/SomeC++Project/build /home/masouye/DriverlessRT/EPFL-RT-Driverless/SomeC++Project/build/test/testfoo /home/masouye/DriverlessRT/EPFL-RT-Driverless/SomeC++Project/build/test/testfoo/CMakeFiles/coverage_testFoo.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/testfoo/CMakeFiles/coverage_testFoo.dir/depend

