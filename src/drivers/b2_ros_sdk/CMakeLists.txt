cmake_minimum_required(VERSION 3.0.2)
project(b2_ros_sdk)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Chosse build type
set(default_build_type "Release")
if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
    set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
            STRING "Choose the type of build." FORCE)
    # Set the possible values of build type for cmake-gui
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
            "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif ()

## Use GNUInstallDirs to install libraries into correct locations on all platforms.
#include(GNUInstallDirs)

## Check system architecture
message(STATUS "Current system architecture: ${CMAKE_SYSTEM_PROCESSOR}")

## Import thirdparty libraries
add_subdirectory(thirdparty)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        geometry_msgs
        sensor_msgs
)

## Import Unitree SDK2 library
set(UNITREE_SDK_PATH ${CMAKE_CURRENT_LIST_DIR}/lib/${CMAKE_SYSTEM_PROCESSOR})
set(UNITREE_SDK_LIB ${UNITREE_SDK_PATH}/libunitree_sdk2.a)

if (NOT EXISTS ${UNITREE_SDK_LIB})
    message(FATAL_ERROR "Unitree SDK library for the architecture is not found")
else ()
    message(STATUS "Unitree SDK library found at: ${UNITREE_SDK_LIB}")
endif ()

message(STATUS "Importing: ${UNITREE_SDK_LIB}")

find_package(Threads REQUIRED)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES b2_ros_sdk
        CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs
        #        DEPENDS unitree_sdk2
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(unitree_sdk2 STATIC IMPORTED GLOBAL)
set_target_properties(unitree_sdk2 PROPERTIES
        IMPORTED_LOCATION ${UNITREE_SDK_LIB})
target_link_libraries(unitree_sdk2 INTERFACE ddsc ddscxx Threads::Threads)
#target_include_directories(unitree_sdk2 INTERFACE
#        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#        $<INSTALL_INTERFACE:include>)

add_executable(b2_stand_example
        src/b2/b2_stand_example.cpp
)
target_link_libraries(b2_stand_example unitree_sdk2)

add_executable(b2_sport_client
        src/b2/b2_sport_client.cpp
)
target_link_libraries(b2_sport_client unitree_sdk2 ${catkin_LIBRARIES})


add_executable(velocity_control
        src/b2/velocity_control.cpp
)
target_link_libraries(velocity_control unitree_sdk2 ${catkin_LIBRARIES})


add_executable(b2_imu_pub
        src/b2/b2_imu_pub.cpp
)
target_link_libraries(b2_imu_pub unitree_sdk2 ${catkin_LIBRARIES})


## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/b2_ros_sdk_node.cpp)

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

### Install the library
#install(DIRECTORY include/
#        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
#
#install(FILES ${UNITREE_SDK_LIB}
#        DESTINATION ${CMAKE_INSTALL_LIBDIR})
#
#install(FILES cmake/unitree_sdk2Targets.cmake
#        DESTINATION lib/cmake/unitree_sdk2)
#
#include(CMakePackageConfigHelpers)
#write_basic_package_version_file(
#        unitree_sdk2ConfigVersion.cmake
#        VERSION "${${PROJECT_NAME}_VERSION_MAJOR}.${${PROJECT_NAME}_VERSION_MINOR}.${${PROJECT_NAME}_VERSION_PATCH}"
#        COMPATIBILITY ExactVersion)
#
#configure_file(cmake/unitree_sdk2Config.cmake.in unitree_sdk2Config.cmake @ONLY)
#install(FILES "${CMAKE_CURRENT_BINARY_DIR}/unitree_sdk2Config.cmake"
#        "${CMAKE_CURRENT_BINARY_DIR}/unitree_sdk2ConfigVersion.cmake"
#        DESTINATION lib/cmake/unitree_sdk2)
#
### Put all binary files into /bin and libraries into /lib
#set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
#set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_BINDIR})
