# BSD 3-clause Copyright (c) 2021, New York University and Max Planck
# Gesellschaft

#
# set up the project
#
cmake_minimum_required(VERSION 3.10.2)

project(mim_control)

# specify the C++ 17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

#
# Search for dependencies.
#

# Local depends
find_package(yaml_utils REQUIRED)
find_package(mpi_cmake_modules REQUIRED)

# External dependencies Extract major/minor python version
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(eiquadprog REQUIRED)
find_package(pybind11 CONFIG REQUIRED)
find_package(Python ${PYTHON_VERSION_STRING} REQUIRED)
# Specific search of boost python as it is yet not trivial.
search_for_boost_python()

# Optionnal
find_package(dynamic-graph QUIET)
find_package(dynamic-graph-python QUIET)

set(build_dynamic_graph_plugins FALSE)
if(${dynamic-graph_FOUND} AND ${dynamic-graph-python_FOUND})
  set(build_dynamic_graph_plugins TRUE)
endif()

#
# Create the main library
#

# cmake-format: off
add_library(
  ${PROJECT_NAME} SHARED
  src/impedance_controller.cpp
  src/centroidal_pd_controller.cpp
  src/centroidal_force_qp_controller.cpp
  src/centroidal_impedance_controller.cpp)
# cmake-format: on
target_link_libraries(${PROJECT_NAME} pinocchio::pinocchio)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
target_link_libraries(${PROJECT_NAME} eiquadprog::eiquadprog)
target_link_libraries(${PROJECT_NAME} yaml_utils::yaml_utils)

# Includes. Add the include dependencies
target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include>)

#
# Build the dynamic graph plugins if the dependencies are found.
#
if(${build_dynamic_graph_plugins})
  # plugin name
  set(plugin_name wbc)
  # Create the library
  add_library(
    ${plugin_name} SHARED
    src/dynamic_graph/signal_utils.cpp
    src/dynamic_graph/impedance_controller.cpp
    src/dynamic_graph/centroidal_pd_controller.cpp
    src/dynamic_graph/centroidal_force_qp_controller.cpp)
  # Add the include dependencies.
  target_include_directories(
    ${plugin_name} PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                          $<INSTALL_INTERFACE:include>
                          ${pybind11_INCLUDE_DIRS})
  # Link the dependencies.
  target_link_libraries(${plugin_name} ${PROJECT_NAME})
  target_link_libraries(${plugin_name} dynamic-graph::dynamic-graph)
  target_link_libraries(${plugin_name}
                        dynamic-graph-python::dynamic-graph-python)
  # Install the target and it's python bindings.
  install_dynamic_graph_plugin_python_bindings(${plugin_name})
  # Install the plugin.
  get_dynamic_graph_plugin_install_path(plugin_install_path)
  install(
    TARGETS ${plugin_name}
    EXPORT ${PROJECT_NAME}Targets
    LIBRARY DESTINATION ${plugin_install_path}
    ARCHIVE DESTINATION ${plugin_install_path}
    RUNTIME DESTINATION ${plugin_install_path}
    INCLUDES
    DESTINATION include)
endif()

#
# Install the package
#

# command to install the library and binaries
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)

# we also need to install the header files
install(DIRECTORY include/ DESTINATION include)

#
# python bindings with pybind11#
#

# cmake-format: off
pybind11_add_module(${PROJECT_NAME}_cpp MODULE
  srcpy/${PROJECT_NAME}.cpp
  srcpy/impedance_controller.cpp
  srcpy/centroidal_pd_controller.cpp
  srcpy/centroidal_force_qp_controller.cpp
  srcpy/centroidal_impedance_controller.cpp)
# cmake-format: on
target_link_libraries(${PROJECT_NAME}_cpp PRIVATE pybind11::module)
target_link_libraries(${PROJECT_NAME}_cpp PRIVATE ${PROJECT_NAME})
target_link_boost_python(${PROJECT_NAME}_cpp PRIVATE)

# install the bindings
get_python_install_dir(PYTHON_INSTALL_DIR)
install(TARGETS ${PROJECT_NAME}_cpp DESTINATION ${PYTHON_INSTALL_DIR})

#
# install python packages
#

# install the python package too
install(
  DIRECTORY python/${PROJECT_NAME}
  DESTINATION "${PYTHON_INSTALL_DIR}"
  PATTERN "*.pyc" EXCLUDE
  PATTERN "__pycache__" EXCLUDE)

#
# building documentation
#
add_documentation()

#
# create the cmake package
#
generate_cmake_package(INSTALL_EXPORT)
