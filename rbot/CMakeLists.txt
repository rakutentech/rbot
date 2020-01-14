cmake_minimum_required(VERSION 2.8.3)
project(rbot)

## Find catkin macros and libraries
find_package(catkin REQUIRED)
catkin_metapacakge()

## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rbot
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)
