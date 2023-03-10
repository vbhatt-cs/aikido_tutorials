cmake_minimum_required(VERSION 3.10.0)
project(aikido_tutorials)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(catkin REQUIRED COMPONENTS
  roscpp
)


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
  LIBRARIES aikido_tutorials
  CATKIN_DEPENDS roscpp
)


# ===============================================================================
# Dependencies
#
include_directories(${catkin_INCLUDE_DIRS})

find_package(DART 6.6.2 REQUIRED
  COMPONENTS utils utils-urdf optimizer-nlopt
)
include_directories(${DART_INCLUDE_DIRS})

find_package(aikido 0.4.0 REQUIRED
  COMPONENTS
    common
    constraint
    control
    control_ros
    distance
    io
    perception
    planner
    planner_kunzretimer
    planner_ompl
    planner_parabolic
    planner_vectorfield
    robot
    robot_ros
    rviz
    statespace
)
include_directories(${aikido_INCLUDE_DIRS})

#================================================================================
# Compiler settings
#
add_compile_options(-Wall -Wextra -Wpedantic)

#==============================================================================
# Build Description
#

include_directories(include)

file(GLOB_RECURSE SOURCES
    LIST_DIRECTORIES false
    src/main.cpp)

add_executable(aikido_tutorials ${SOURCES})

target_link_libraries(aikido_tutorials
  ${DART_LIBRARIES}
  ${aikido_LIBRARIES})

install(TARGETS aikido_tutorials
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

# ===============================================================================
# Formatting
#

include(ClangFormat)
clang_format_setup(VERSION 10)

if(CLANG_FORMAT_EXECUTABLE)

  file(GLOB_RECURSE ALL_SOURCE_FILES
    LIST_DIRECTORIES false
    include/*.hpp
    src/*.cpp)

  clang_format_add_sources(${ALL_SOURCE_FILES})

  clang_format_add_targets()
endif()
