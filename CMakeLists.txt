cmake_minimum_required(VERSION 3.0.2)
project(lio)

## Compile as C++11, supported in ROS Kinetic and newer

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -zi -Wall -DEBUG -march=native")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(PYTHON_EXECUTABLE "/usr/bin/python3")
set(CMAKE_VERBOSE_MAKEFILE true)
#add_compile_options(-march=native)
#include(FetchContent)
#FetchContent_Declare(
#        googletest
#        URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
#)
# For Windows: Prevent overriding the parent project's compiler/linker settings
#set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
#FetchContent_MakeAvailable(googletest)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        sensor_msgs
        geometry_msgs
        nav_msgs
        pcl_ros
        message_generation
        livox_ros_driver
        )

#find_package(PCL REQUIRED)
#include_directories(${PCL_INCLUDE_DIRS})
#add_definitions(${PCL_DEFINITIONS})
#link_directories(${PCL_LIBRARY_DIRS})
find_package(PCL 1.8 REQUIRED)
find_package(Eigen3 REQUIRED)
add_definitions(${EIGEN_DEFINITIONS})
include_directories(${EIGEN3_INCLUDE_DIR})
find_package(Sophus REQUIRED)
INCLUDE_DIRECTORIES(${Sophus_INCLUDE_DIRS})
add_definitions("-D SOPHUS_USE_BASIC_LOGGING")

find_package(Python3 COMPONENTS Development NumPy)

find_package(glog 0.7.0 REQUIRED)

find_package(Python3 COMPONENTS Development NumPy)

find_package(TBB REQUIRED)

include(FindOpenMP)
if (OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
else (OPENMP_FOUND)
    message("ERROR: OpenMP could not be found.")
endif (OPENMP_FOUND)
FIND_PACKAGE(OpenCV REQUIRED)
find_package(Threads REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES lio
        #  CATKIN_DEPENDS other_catkin_pkg
        #  DEPENDS system_lib
        CATKIN_DEPENDS
        roscpp
        std_msgs
        sensor_msgs
        geometry_msgs
        pcl_ros
        INCLUDE_DIRS
        include
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/lio.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/lio_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
add_executable(test_pc_fields src/test_pc_fields.cpp)
target_link_libraries(test_pc_fields ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenMP_LIBS} Threads::Threads)
#add_executable(odom src/odom.cpp)
#target_link_libraries(odom ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenMP_LIBS} Threads::Threads)
#enable_testing()

add_executable(
        test_odom
        src/test_odom.cpp
        src/odom.cpp
        src/timer.cpp
        src/odom_v2.cpp
        src/voxelmap.cpp
)
target_link_libraries(
        test_odom
        ${catkin_LIBRARIES} ${PCL_LIBRARIES} glog::glog TBB::tbb
)
add_executable(lo src/LO.cpp src/odom.cpp src/timer.cpp)
target_link_libraries(lo ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PYTHON3_LIBRARIES} Python3::Python Python3::NumPy glog TBB::tbb)
target_include_directories(lo PRIVATE ${PYTHON_INCLUDE_DIRS} ${Python3_NumPy_INCLUDE_DIRS})
#include(GoogleTest)
#gtest_discover_tests(test_odom)
add_executable(scripts2TUM src/csv2txtwithTUM.cpp)
target_link_libraries(scripts2TUM ${catkin_LIBRARIES} ${OpenCV_LIBS})


add_executable(lo_v2 src/lo_v2.cpp src/odom_v2.cpp src/voxelmap_v2.cpp src/timer.cpp src/imu.cpp)
target_link_libraries(lo_v2 ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PYTHON3_LIBRARIES} Python3::Python Python3::NumPy glog::glog TBB::tbb)
target_include_directories(lo_v2 PRIVATE ${PYTHON_INCLUDE_DIRS} ${Python3_NumPy_INCLUDE_DIRS})

add_executable(lo_v3 src/lo_v3.cpp src/odom_v3.cpp src/voxelmap_v2.cpp src/timer.cpp src/imu.cpp)
target_link_libraries(lo_v3 ${catkin_LIBRARIES} TBB::tbb)

add_executable(lo_v4 src/lo_v4.cpp src/odom_v4.cpp src/voxelmap_v2.cpp src/timer.cpp src/imu.cpp)
target_link_libraries(lo_v4 ${catkin_LIBRARIES} TBB::tbb)

#target_include_directories(lo_v3 PRIVATE ${PYTHON_INCLUDE_DIRS} ${Python3_NumPy_INCLUDE_DIRS})
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_lio.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
