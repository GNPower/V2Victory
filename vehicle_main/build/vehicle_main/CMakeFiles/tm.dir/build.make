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
CMAKE_SOURCE_DIR = /home/ubuntu/V2Victory/vehicle_main

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ubuntu/V2Victory/vehicle_main/build/vehicle_main

# Include any dependencies generated for this target.
include CMakeFiles/tm.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/tm.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/tm.dir/flags.make

CMakeFiles/tm.dir/test_main.cpp.o: CMakeFiles/tm.dir/flags.make
CMakeFiles/tm.dir/test_main.cpp.o: ../../test_main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/V2Victory/vehicle_main/build/vehicle_main/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/tm.dir/test_main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/tm.dir/test_main.cpp.o -c /home/ubuntu/V2Victory/vehicle_main/test_main.cpp

CMakeFiles/tm.dir/test_main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tm.dir/test_main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/V2Victory/vehicle_main/test_main.cpp > CMakeFiles/tm.dir/test_main.cpp.i

CMakeFiles/tm.dir/test_main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tm.dir/test_main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/V2Victory/vehicle_main/test_main.cpp -o CMakeFiles/tm.dir/test_main.cpp.s

CMakeFiles/tm.dir/encoder/encoder.cpp.o: CMakeFiles/tm.dir/flags.make
CMakeFiles/tm.dir/encoder/encoder.cpp.o: ../../encoder/encoder.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/V2Victory/vehicle_main/build/vehicle_main/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/tm.dir/encoder/encoder.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/tm.dir/encoder/encoder.cpp.o -c /home/ubuntu/V2Victory/vehicle_main/encoder/encoder.cpp

CMakeFiles/tm.dir/encoder/encoder.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tm.dir/encoder/encoder.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/V2Victory/vehicle_main/encoder/encoder.cpp > CMakeFiles/tm.dir/encoder/encoder.cpp.i

CMakeFiles/tm.dir/encoder/encoder.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tm.dir/encoder/encoder.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/V2Victory/vehicle_main/encoder/encoder.cpp -o CMakeFiles/tm.dir/encoder/encoder.cpp.s

CMakeFiles/tm.dir/localize/localize.cpp.o: CMakeFiles/tm.dir/flags.make
CMakeFiles/tm.dir/localize/localize.cpp.o: ../../localize/localize.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/V2Victory/vehicle_main/build/vehicle_main/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/tm.dir/localize/localize.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/tm.dir/localize/localize.cpp.o -c /home/ubuntu/V2Victory/vehicle_main/localize/localize.cpp

CMakeFiles/tm.dir/localize/localize.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tm.dir/localize/localize.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/V2Victory/vehicle_main/localize/localize.cpp > CMakeFiles/tm.dir/localize/localize.cpp.i

CMakeFiles/tm.dir/localize/localize.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tm.dir/localize/localize.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/V2Victory/vehicle_main/localize/localize.cpp -o CMakeFiles/tm.dir/localize/localize.cpp.s

CMakeFiles/tm.dir/motor_drive/motor_drive.cpp.o: CMakeFiles/tm.dir/flags.make
CMakeFiles/tm.dir/motor_drive/motor_drive.cpp.o: ../../motor_drive/motor_drive.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/V2Victory/vehicle_main/build/vehicle_main/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/tm.dir/motor_drive/motor_drive.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/tm.dir/motor_drive/motor_drive.cpp.o -c /home/ubuntu/V2Victory/vehicle_main/motor_drive/motor_drive.cpp

CMakeFiles/tm.dir/motor_drive/motor_drive.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tm.dir/motor_drive/motor_drive.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/V2Victory/vehicle_main/motor_drive/motor_drive.cpp > CMakeFiles/tm.dir/motor_drive/motor_drive.cpp.i

CMakeFiles/tm.dir/motor_drive/motor_drive.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tm.dir/motor_drive/motor_drive.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/V2Victory/vehicle_main/motor_drive/motor_drive.cpp -o CMakeFiles/tm.dir/motor_drive/motor_drive.cpp.s

# Object files for target tm
tm_OBJECTS = \
"CMakeFiles/tm.dir/test_main.cpp.o" \
"CMakeFiles/tm.dir/encoder/encoder.cpp.o" \
"CMakeFiles/tm.dir/localize/localize.cpp.o" \
"CMakeFiles/tm.dir/motor_drive/motor_drive.cpp.o"

# External object files for target tm
tm_EXTERNAL_OBJECTS =

tm: CMakeFiles/tm.dir/test_main.cpp.o
tm: CMakeFiles/tm.dir/encoder/encoder.cpp.o
tm: CMakeFiles/tm.dir/localize/localize.cpp.o
tm: CMakeFiles/tm.dir/motor_drive/motor_drive.cpp.o
tm: CMakeFiles/tm.dir/build.make
tm: /opt/ros/foxy/lib/librclcpp.so
tm: ../../car_interface/install/car_interface/lib/libcar_interface__rosidl_typesupport_introspection_c.so
tm: ../../car_interface/install/car_interface/lib/libcar_interface__rosidl_typesupport_c.so
tm: ../../car_interface/install/car_interface/lib/libcar_interface__rosidl_typesupport_introspection_cpp.so
tm: ../../car_interface/install/car_interface/lib/libcar_interface__rosidl_typesupport_cpp.so
tm: /opt/ros/foxy/lib/liblibstatistics_collector.so
tm: /opt/ros/foxy/lib/liblibstatistics_collector_test_msgs__rosidl_typesupport_introspection_c.so
tm: /opt/ros/foxy/lib/liblibstatistics_collector_test_msgs__rosidl_generator_c.so
tm: /opt/ros/foxy/lib/liblibstatistics_collector_test_msgs__rosidl_typesupport_c.so
tm: /opt/ros/foxy/lib/liblibstatistics_collector_test_msgs__rosidl_typesupport_introspection_cpp.so
tm: /opt/ros/foxy/lib/liblibstatistics_collector_test_msgs__rosidl_typesupport_cpp.so
tm: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_introspection_c.so
tm: /opt/ros/foxy/lib/libstd_msgs__rosidl_generator_c.so
tm: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_c.so
tm: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_introspection_cpp.so
tm: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_cpp.so
tm: /opt/ros/foxy/lib/librcl.so
tm: /opt/ros/foxy/lib/librcl_interfaces__rosidl_typesupport_introspection_c.so
tm: /opt/ros/foxy/lib/librcl_interfaces__rosidl_generator_c.so
tm: /opt/ros/foxy/lib/librcl_interfaces__rosidl_typesupport_c.so
tm: /opt/ros/foxy/lib/librcl_interfaces__rosidl_typesupport_introspection_cpp.so
tm: /opt/ros/foxy/lib/librcl_interfaces__rosidl_typesupport_cpp.so
tm: /opt/ros/foxy/lib/librmw_implementation.so
tm: /opt/ros/foxy/lib/librmw.so
tm: /opt/ros/foxy/lib/librcl_logging_spdlog.so
tm: /usr/lib/aarch64-linux-gnu/libspdlog.so.1.5.0
tm: /opt/ros/foxy/lib/librcl_yaml_param_parser.so
tm: /opt/ros/foxy/lib/libyaml.so
tm: /opt/ros/foxy/lib/librosgraph_msgs__rosidl_typesupport_introspection_c.so
tm: /opt/ros/foxy/lib/librosgraph_msgs__rosidl_generator_c.so
tm: /opt/ros/foxy/lib/librosgraph_msgs__rosidl_typesupport_c.so
tm: /opt/ros/foxy/lib/librosgraph_msgs__rosidl_typesupport_introspection_cpp.so
tm: /opt/ros/foxy/lib/librosgraph_msgs__rosidl_typesupport_cpp.so
tm: /opt/ros/foxy/lib/libstatistics_msgs__rosidl_typesupport_introspection_c.so
tm: /opt/ros/foxy/lib/libstatistics_msgs__rosidl_generator_c.so
tm: /opt/ros/foxy/lib/libstatistics_msgs__rosidl_typesupport_c.so
tm: /opt/ros/foxy/lib/libstatistics_msgs__rosidl_typesupport_introspection_cpp.so
tm: /opt/ros/foxy/lib/libstatistics_msgs__rosidl_typesupport_cpp.so
tm: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_c.so
tm: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_generator_c.so
tm: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_c.so
tm: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_cpp.so
tm: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_cpp.so
tm: /opt/ros/foxy/lib/libtracetools.so
tm: ../../car_interface/install/car_interface/lib/libcar_interface__rosidl_generator_c.so
tm: /opt/ros/foxy/lib/librosidl_typesupport_introspection_cpp.so
tm: /opt/ros/foxy/lib/librosidl_typesupport_introspection_c.so
tm: /opt/ros/foxy/lib/librosidl_typesupport_cpp.so
tm: /opt/ros/foxy/lib/librosidl_typesupport_c.so
tm: /opt/ros/foxy/lib/librcpputils.so
tm: /opt/ros/foxy/lib/librosidl_runtime_c.so
tm: /opt/ros/foxy/lib/librcutils.so
tm: CMakeFiles/tm.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ubuntu/V2Victory/vehicle_main/build/vehicle_main/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX executable tm"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/tm.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/tm.dir/build: tm

.PHONY : CMakeFiles/tm.dir/build

CMakeFiles/tm.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/tm.dir/cmake_clean.cmake
.PHONY : CMakeFiles/tm.dir/clean

CMakeFiles/tm.dir/depend:
	cd /home/ubuntu/V2Victory/vehicle_main/build/vehicle_main && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ubuntu/V2Victory/vehicle_main /home/ubuntu/V2Victory/vehicle_main /home/ubuntu/V2Victory/vehicle_main/build/vehicle_main /home/ubuntu/V2Victory/vehicle_main/build/vehicle_main /home/ubuntu/V2Victory/vehicle_main/build/vehicle_main/CMakeFiles/tm.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/tm.dir/depend

