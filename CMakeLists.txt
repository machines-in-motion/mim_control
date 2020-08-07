# BSD 3-clause
# Copyright (c) 2020, New York University and Max Planck Gesellschaft
cmake_minimum_required(VERSION 3.5.1)

project(blmc_controllers)

# required to use std::shared_ptr in code and to link the python bindings
if(NOT ${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,--no-as-needed")
endif()
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -W -Wextra")

# ensuring path to libraries are set during install
set(CMAKE_SKIP_BUILD_RPATH  FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib)

############################
# Search for dependencies. #
############################

set(CATKIN_PKGS ${CATKIN_PKGS}
    # MPI-IS dependencies
    mpi_cmake_modules
    pybind11_catkin
)
find_package(catkin REQUIRED COMPONENTS ${CATKIN_PKGS})

# catkin customised search macros.
SET(BOOST_COMPONENTS
      filesystem system thread program_options unit_test_framework python)
search_for_python()
search_for_boost()
search_for_eigen()
catkin_add_required_dependency("pinocchio >= 2.0")

######################################################
# define the include directory of all ${CATKIN_PKGS} #
######################################################
include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
)

##########################################
# export the package as a catkin package #
##########################################
set(library_list ${PROJECT_NAME})
catkin_package(
    LIBRARIES ${library_list}
    # INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include}
    CATKIN_DEPENDS ${CATKIN_PKGS}
)

##############################
# manage python file install #
##############################
catkin_python_setup()

###########################
# Create the main library #
###########################

add_library(${PROJECT_NAME}
    src/impedance_controller.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

################################
# python bindings with pybind11#
################################
pybind11_add_module(py_${PROJECT_NAME}
    srcpy/${PROJECT_NAME}.cpp
    srcpy/impedance_controller.cpp
)
target_link_libraries(
    py_${PROJECT_NAME} PRIVATE ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_PYTHON_LIBRARY})
set_target_properties(py_${PROJECT_NAME} PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION}
  OUTPUT_NAME ${PROJECT_NAME}
)
install(TARGETS py_${PROJECT_NAME} DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION})


##########################
# building documentation #
##########################
add_sphinx_documentation()