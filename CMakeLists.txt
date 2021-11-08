cmake_minimum_required(VERSION 3.5)
project(stero_mobile_init)
set(CMAKE_CXX_STANDARD 11)
find_package(catkin REQUIRED
    COMPONENTS
    std_msgs
    tf2_ros
    # without things below i get many lines like this:
    # navi.cpp:(.text+0x6fc): undefined reference to `dwa_local_planner::DWAPlannerROS::DWAPlannerROS()'

    # and with these things i get many lines like this:
    # CMake Error at /home/mpalczuk/mobile_ws/build/navigation/navfn/cmake/navfn-genmsg.cmake:221 (add_custom_target):
    # add_custom_target cannot create target "navfn_generate_messages_py" because
    # another target with the same name already exists.  The existing target is a
    # custom target created in source directory

    costmap_2d
    dwa_local_planner
    global_planner
    rotate_recovery
)
catkin_package(
    # INCLUDE_DIRS include
    # LIBRARIES ${PROJECT_NAME}
    # CATKIN_DEPENDS roscpp nodelet
    # DEPENDS elektron
)
include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(navi scripts/navi.cpp)
target_link_libraries(navi ${catkin_LIBRARIES})

install (DIRECTORY worlds/ DESTINATION share/${CMAKE_PROJECT_NAME}/worlds 
         FILES_MATCHING PATTERN "*.world")
install (DIRECTORY launch/ DESTINATION share/${CMAKE_PROJECT_NAME}/launch 
         FILES_MATCHING PATTERN "*.launch*")
install (DIRECTORY config/ DESTINATION share/${CMAKE_PROJECT_NAME}/config 
         FILES_MATCHING PATTERN "*.yaml")
install (DIRECTORY param/ DESTINATION share/${CMAKE_PROJECT_NAME}/param 
         FILES_MATCHING PATTERN "*.yaml")
