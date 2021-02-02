# Copyright (C) 2008-2019 LAAS-CNRS, JRL AIST-CNRS, INRIA
#
# This program is free software: you can redistribute it and/or modify it under
# the terms of the GNU General Public License as published by the Free Software
# Foundation, either version 3 of the License, or (at your option) any later
# version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
# FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
# details.
#
# You should have received a copy of the GNU General Public License along with
# this program.  If not, see <http://www.gnu.org/licenses/>.

# .rst: .. command:: SEARCH_FOR_BOOST_COMPONENT
#
# :param boost_python_name: :param found:
#
# This function returns found to TRUE if the boost_python_name has been found,
# FALSE otherwise. This function is for internal use only.
#
function(SEARCH_FOR_BOOST_COMPONENT boost_python_name found)
  set(${found}
      FALSE
      PARENT_SCOPE)
  find_package(Boost ${BOOST_REQUIRED} QUIET
               OPTIONAL_COMPONENTS ${boost_python_name})
  string(TOUPPER ${boost_python_name} boost_python_name_UPPER)
  if(Boost_${boost_python_name_UPPER}_FOUND)
    set(${found}
        TRUE
        PARENT_SCOPE)
  endif()
endfunction(
  SEARCH_FOR_BOOST_COMPONENT
  boost_python_name
  found)

if(CMAKE_VERSION VERSION_LESS "3.12")
  set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/boost ${CMAKE_MODULE_PATH})
  message(
    STATUS
      "CMake versions older than 3.12 may warn when looking to Boost components. Custom macros are used to find it."
  )
endif(CMAKE_VERSION VERSION_LESS "3.12")

# .rst: .. command:: SET_BOOST_DEFAULT_OPTIONS
#
# This function allows to set up the default options for detecting Boost
# components.
#
macro(SET_BOOST_DEFAULT_OPTIONS)
  set(Boost_USE_STATIC_LIBS OFF)
  set(Boost_USE_MULTITHREADED ON)
  set(Boost_NO_BOOST_CMAKE ON)
endmacro(SET_BOOST_DEFAULT_OPTIONS)

# .rst: .. command:: EXPORT_BOOST_DEFAULT_OPTIONS
#
# This function allows to export the default options for detecting Boost
# components.
#
macro(EXPORT_BOOST_DEFAULT_OPTIONS)
  list(
    INSERT
    _PACKAGE_CONFIG_DEPENDENCIES_FIND_PACKAGE
    0
    "SET(Boost_USE_STATIC_LIBS OFF);SET(Boost_USE_MULTITHREADED ON);SET(Boost_NO_BOOST_CMAKE ON)"
  )
  list(
    INSERT
    _PACKAGE_CONFIG_DEPENDENCIES_FIND_DEPENDENCY
    0
    "SET(Boost_USE_STATIC_LIBS OFF);SET(Boost_USE_MULTITHREADED ON);SET(Boost_NO_BOOST_CMAKE ON)"
  )
endmacro(EXPORT_BOOST_DEFAULT_OPTIONS)

#
# .rst .. command:: SEARCH_FOR_BOOST_PYTHON([REQUIRED])
#
# Find boost-python component. For boost >= 1.67.0, FindPython macro should be
# called first in order to automatically detect the right boost-python component
# version according to the Python version (2.7 or 3.x).
#

macro(SEARCH_FOR_BOOST_PYTHON)

  cmake_parse_arguments(_BOOST_PYTHON_REQUIRED "REQUIRED" "" "" ${ARGN})
  set(BOOST_PYTHON_NAME "python")
  set(BOOST_PYTHON_REQUIRED "")
  if(_BOOST_PYTHON_REQUIRED)
    set(BOOST_PYTHON_REQUIRED REQUIRED)
  endif(_BOOST_PYTHON_REQUIRED)

  set_boost_default_options()

  if(NOT PYTHONLIBS_FOUND)
    message(
      FATAL_ERROR
        "Python has not been found. You should first call FindPython before calling SEARCH_FOR_BOOST_PYTHON macro."
    )
  endif(NOT PYTHONLIBS_FOUND)

  # Test: pythonX, pythonXY and python-pyXY
  set(BOOST_PYTHON_COMPONENT_LIST
      "python${PYTHON_VERSION_MAJOR}"
      "python${PYTHON_VERSION_MAJOR}${PYTHON_VERSION_MINOR}"
      "python-py${PYTHON_VERSION_MAJOR}${PYTHON_VERSION_MINOR}")

  set(BOOST_PYTHON_FOUND FALSE)
  foreach(BOOST_PYTHON_COMPONENT ${BOOST_PYTHON_COMPONENT_LIST})
    search_for_boost_component(${BOOST_PYTHON_COMPONENT} BOOST_PYTHON_FOUND)
    if(BOOST_PYTHON_FOUND)
      set(BOOST_PYTHON_NAME ${BOOST_PYTHON_COMPONENT})
      break()
    endif(BOOST_PYTHON_FOUND)
  endforeach(BOOST_PYTHON_COMPONENT ${BOOST_PYTHON_COMPONENT_LIST})

  # If boost-python has not been found, warn the user, and look for just
  # "python"
  if(NOT BOOST_PYTHON_FOUND)
    message(
      WARNING "Impossible to check Boost.Python version. Trying with 'python'.")
  endif(NOT BOOST_PYTHON_FOUND)

  find_package(Boost ${BOOST_PYTHON_REQUIRED} COMPONENTS ${BOOST_PYTHON_NAME})
  string(TOUPPER ${BOOST_PYTHON_NAME} UPPERCOMPONENT)

  list(APPEND LOGGING_WATCHED_VARIABLES Boost_${UPPERCOMPONENT}_FOUND
       Boost_${UPPERCOMPONENT}_LIBRARY Boost_${UPPERCOMPONENT}_LIBRARY_DEBUG
       Boost_${UPPERCOMPONENT}_LIBRARY_RELEASE)

  set(Boost_PYTHON_LIBRARY ${Boost_${UPPERCOMPONENT}_LIBRARY})
  message(STATUS "Boost_PYTHON_LIBRARY: ${Boost_PYTHON_LIBRARY}")
  list(APPEND Boost_PYTHON_LIBRARIES ${Boost_PYTHON_LIBRARY})
  list(APPEND LOGGING_WATCHED_VARIABLES Boost_PYTHON_LIBRARY)
endmacro(SEARCH_FOR_BOOST_PYTHON)

# .rst: .. command:: TARGET_LINK_BOOST_PYTHON (TARGET
# <PRIVATE|PUBLIC|INTERFACE>)
#
# Link target againt boost_python library.
#
# :target: is either a library or an executable :private,public,interface: The
# PUBLIC, PRIVATE and INTERFACE keywords can be used to specify both the link
# dependencies and the link interface.
#
# On darwin systems, boost_python is not linked against any python library. This
# linkage is resolved at execution time via the python interpreter. We then need
# to stipulate that boost_python has unresolved symbols at compile time for a
# library target. Otherwise, for executables we need to link to a specific
# version of python.
#
macro(TARGET_LINK_BOOST_PYTHON target)
  if(${ARGC} GREATER 1)
    set(PUBLIC_KEYWORD ${ARGV1})
  endif()

  if(APPLE)
    get_target_property(TARGET_TYPE ${target} TYPE)

    if(${TARGET_TYPE} MATCHES EXECUTABLE)
      target_link_libraries(${target} ${PUBLIC_KEYWORD} ${Boost_PYTHON_LIBRARY})
    else(${TARGET_TYPE} MATCHES EXECUTABLE)
      target_link_libraries(
        ${target} ${PUBLIC_KEYWORD}
        -Wl,-undefined,dynamic_lookup,${Boost_PYTHON_LIBRARIES})
    endif(${TARGET_TYPE} MATCHES EXECUTABLE)

  else(APPLE)
    target_link_libraries(${target} ${PUBLIC_KEYWORD} ${Boost_PYTHON_LIBRARY})
  endif(APPLE)
  list(APPEND LOGGING_WATCHED_VARIABLES Boost_PYTHON_LIBRARIES)
endmacro(TARGET_LINK_BOOST_PYTHON)
