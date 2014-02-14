cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

set(USE_SIMULATION false CACHE BOOL "Use fake library instead of real robot")

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()


if (USE_SIMULATION)

	set(VERBOSE false CACHE BOOL "Verbose simulation")
	set(SILENT false CACHE BOOL "Silent simulation")

	if (VERBOSE)
		add_definitions(-DVERBOSE)
	else (VERBOSE)
		remove_definitions(-DVERBOSE)
	endif (VERBOSE)
	if (SILENT)
		add_definitions(-DSILENT)
	else (SILENT)
		remove_definitions(-DSILENT)
	endif (SILENT)

	rosbuild_add_library(nifti_robot_simulation src/nifti_robot_simulation.c)
endif (USE_SIMULATION)

#rosbuild_add_library(nifti_robot src/nifti_robot.cpp)
rosbuild_add_executable(nifti_robot_node src/nifti_robot.cpp src/nifti_robot_main.cpp)

if (USE_SIMULATION)
	target_link_libraries(nifti_robot_node nifti_robot_simulation)
else (USE_SIMULATION)
	target_link_libraries(nifti_robot_node rover)
endif (USE_SIMULATION)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
