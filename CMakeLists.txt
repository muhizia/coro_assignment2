##################################################################################
#
# Customize this file  by replacing amuhizi with your own student ID:
# there are six replacement to be made.
#
# Make sure you have done the following:
#
# - created a package named assignment1
#   and that you use this make file in that package
#
# - created a subdirectory named data in the package
#   and that you have created the input file assignment1Input.txt there
#
# - created the interface file amuhizi.h in the include/assignment1 subdirectory
#
# - created the application source file amuhiziApplication.cpp
#   and the implementation source file  amuhiziImplementation.cpp
#   in the src subdirectory
#
##################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(assignment2)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  #sensor_msgs
  #std_msgs
  #tf
  roslib
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} include)

# Install data files
install(DIRECTORY data/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/data)

add_executable       (${PROJECT_NAME}_amuhizi src/amuhiziImplementation.cpp src/amuhiziApplication.cpp)
set_target_properties(${PROJECT_NAME}_amuhizi PROPERTIES OUTPUT_NAME amuhizi  PREFIX "")
target_link_libraries(${PROJECT_NAME}_amuhizi ${catkin_LIBRARIES})
