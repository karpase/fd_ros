cmake_minimum_required(VERSION 2.8.3)
project(fd_ros)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  genmsg
  roscpp
  rospy
  std_msgs  
  message_generation
)

add_message_files(FILES Fact.msg)
add_service_files(FILES Planning.srv)
generate_messages()


catkin_package(
   CATKIN_DEPENDS message_runtime)





# Respect the PATH environment variable when searching for compilers.
find_program(CMAKE_C_COMPILER NAMES $ENV{CC} gcc PATHS ENV PATH NO_DEFAULT_PATH)
find_program(CMAKE_CXX_COMPILER NAMES $ENV{CXX} g++ PATHS ENV PATH NO_DEFAULT_PATH)

# Path containing custom CMake modules
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules)
include(FastDownwardMacros)

fast_downward_default_to_release_build()
# We have to set the desired bitwidth before we can define the project.
fast_downward_set_bitwidth()
project(fast-downward)
#fast_downward_check_64_bit_option()
# Due to a bug in cmake, configuration types are only set up correctly on the second cmake run.
# This means that cmake has to be called twice for multi-config generators like Visual Studio.
fast_downward_set_configuration_types()
fast_downward_add_profile_build()

set(FAST_DOWNWARD_MAIN_CMAKELISTS_READ TRUE)

#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin)


# Add planner components as subprojects.

# Copy the translator into the output directory.
#add_custom_target(translate ALL)
#add_custom_command(TARGET translate POST_BUILD
#    COMMAND ${CMAKE_COMMAND} -E copy_directory
#        ${CMAKE_CURRENT_SOURCE_DIR}/translate
#        ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${CMAKE_CFG_INTDIR}/translate
#    COMMENT "Copying translator module into output directory")

add_subdirectory(search)
