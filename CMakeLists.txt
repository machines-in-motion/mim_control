# BSD 3-clause
# Copyright (c) 2021, New York University and Max Planck Gesellschaft

#
# set up the project
#
cmake_minimum_required(VERSION 3.10.2)

project(blmc_controllers)

# specify the C++ 17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Some additionnal optimization for the code.
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -funroll-loops -fPIC")

############################
# Search for dependencies. #
############################

#
# Dependencies
#
find_package(pybind11 CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(eiquadprog REQUIRED)

find_package(PythonLibs REQUIRED)
# Extract major/minor python version
string(REPLACE "." ";" VERSION_LIST ${PYTHONLIBS_VERSION_STRING})
list(GET VERSION_LIST 0 PYTHONLIBS_VERSION_MAJOR)
list(GET VERSION_LIST 1 PYTHONLIBS_VERSION_MINOR)
find_package(Boost COMPONENTS python${PYTHONLIBS_VERSION_MAJOR}${PYTHONLIBS_VERSION_MINOR} REQUIRED)

#local depends
find_package(mpi_cmake_modules REQUIRED)
find_package(yaml_utils REQUIRED)


###########################
# Create the main library #
###########################

add_library(${PROJECT_NAME} SHARED
    src/impedance_controller.cpp
    src/centroidal_pd_controller.cpp
    src/centroidal_force_qp_controller.cpp
)
target_link_libraries(${PROJECT_NAME} pinocchio::pinocchio)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
target_link_libraries(${PROJECT_NAME} eiquadprog::eiquadprog)
target_link_libraries(${PROJECT_NAME} yaml_utils::yaml_utils)

# Includes. Add the include dependencies
target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include>)

################################
# Install the package          #
################################

# command to install the library and binaries
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include
)

# we also need to install the header files
install(DIRECTORY include/ DESTINATION include)

# ################################
# # python bindings with pybind11#
# ################################
pybind11_add_module(py_${PROJECT_NAME} MODULE
    srcpy/${PROJECT_NAME}.cpp
    srcpy/impedance_controller.cpp
    srcpy/centroidal_pd_controller.cpp
    srcpy/centroidal_force_qp_controller.cpp
)


target_link_libraries(py_${PROJECT_NAME} PRIVATE pybind11::module)
target_link_libraries(py_${PROJECT_NAME} PRIVATE ${PROJECT_NAME})
target_link_libraries(py_${PROJECT_NAME} PRIVATE ${Boost_PYTHON_LIBRARY})

# install the bindings
get_python_install_dir(PYTHON_INSTALL_DIR)
install(TARGETS py_${PROJECT_NAME} DESTINATION ${PYTHON_INSTALL_DIR})

################################
# install python packages      #
################################

# install the python package too
install(
    DIRECTORY python/${PROJECT_NAME}
    DESTINATION "${PYTHON_INSTALL_DIR}"
    PATTERN "*.pyc" EXCLUDE
    PATTERN "__pycache__" EXCLUDE
  )

##########################
# building documentation #
##########################
add_documentation()

############################
# create the cmake package #
############################
generate_cmake_package(INSTALL_EXPORT)
