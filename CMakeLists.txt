cmake_minimum_required(VERSION 3.5.1)
project(controlNode)

find_package(catkin REQUIRED COMPONENTS
 
 roscpp
 rospy
 std_msgs
 core_octomap_map_representation
 core_map_representation_interface
 tflib
 pcl_ros
 pcl_conversions
 core_planning_state_space

)

find_package(ompl REQUIRED)
find_package(PCL REQUIRED)


SET(CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CMAKE_CXX_EXTENSIONS OFF)

file(GLOB_RECURSE SRCS RELATIVE ${PROJECT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

set(PRIVATE_COMPILE_OPTS )

catkin_package(
    LIBRARIES
        controlNode
    INCLUDE_DIRS
        include 
    CATKIN_DEPENDS roscpp rospy std_msgs pcl_ros pcl_conversions
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${OMPL_INCLUDE_DIRS}
 ${PCL_INCLUDE_DIRS}
        ${DYNAMICEDT3D_INCLUDE_DIRS}
)

add_executable(controlNode ${SRCS})

add_dependencies(controlNode ${catkin_EXPORTED_TARGETS})
 add_dependencies(controlNode ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_include_directories(
    controlNode PRIVATE 
    ${PROJECT_SOURCE_DIR/include}
)

 target_link_libraries(controlNode
   ${catkin_LIBRARIES}
         ${OMPL_LIBRARIES}
         ${PCL_LIBRARIES} 
 )
target_compile_options(controlNode PRIVATE ${PRIVATE_COMPILE_OPTS} "-fpermissive")