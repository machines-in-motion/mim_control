# Copyright (C) 2008-2019 LAAS-CNRS, JRL AIST-CNRS, INRIA
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

#.rst:
# .. command:: SEARCH_FOR_BOOST_COMPONENT
#
#   :param boost_python_name:
#   :param found:
#
#  This function returns found to TRUE if the boost_python_name has been found, FALSE otherwise.
#  This function is for internal use only.
#
FUNCTION(SEARCH_FOR_BOOST_COMPONENT boost_python_name found)
  SET(${found} FALSE PARENT_SCOPE)
  FIND_PACKAGE(Boost ${BOOST_REQUIRED} QUIET OPTIONAL_COMPONENTS ${boost_python_name})
  STRING(TOUPPER ${boost_python_name} boost_python_name_UPPER)
  IF(Boost_${boost_python_name_UPPER}_FOUND)
    SET(${found} TRUE PARENT_SCOPE)
  ENDIF()
ENDFUNCTION(SEARCH_FOR_BOOST_COMPONENT boost_python_name found)

IF(CMAKE_VERSION VERSION_LESS "3.12")
  SET(CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/boost ${CMAKE_MODULE_PATH})
  MESSAGE(STATUS "CMake versions older than 3.12 may warn when looking to Boost components. Custom macros are used to find it.")
ENDIF(CMAKE_VERSION VERSION_LESS "3.12")

#.rst:
# .. command:: SET_BOOST_DEFAULT_OPTIONS
#
#  This function allows to set up the default options for detecting Boost components.
# 
MACRO(SET_BOOST_DEFAULT_OPTIONS)
  SET(Boost_USE_STATIC_LIBS OFF)
  SET(Boost_USE_MULTITHREADED ON)
  SET(Boost_NO_BOOST_CMAKE ON) 
ENDMACRO(SET_BOOST_DEFAULT_OPTIONS)

#.rst:
# .. command:: EXPORT_BOOST_DEFAULT_OPTIONS
#
#  This function allows to export the default options for detecting Boost components.
# 
MACRO(EXPORT_BOOST_DEFAULT_OPTIONS)
  LIST(INSERT _PACKAGE_CONFIG_DEPENDENCIES_FIND_PACKAGE 0 "SET(Boost_USE_STATIC_LIBS OFF);SET(Boost_USE_MULTITHREADED ON);SET(Boost_NO_BOOST_CMAKE ON)")
  LIST(INSERT _PACKAGE_CONFIG_DEPENDENCIES_FIND_DEPENDENCY 0 "SET(Boost_USE_STATIC_LIBS OFF);SET(Boost_USE_MULTITHREADED ON);SET(Boost_NO_BOOST_CMAKE ON)")
ENDMACRO(EXPORT_BOOST_DEFAULT_OPTIONS)

#
#.rst
# .. command:: SEARCH_FOR_BOOST_PYTHON([REQUIRED])
#
#  Find boost-python component.
#  For boost >= 1.67.0, FindPython macro should be called first in order
#  to automatically detect the right boost-python component version according
#  to the Python version (2.7 or 3.x).
#

MACRO(SEARCH_FOR_BOOST_PYTHON)

  CMAKE_PARSE_ARGUMENTS(_BOOST_PYTHON_REQUIRED "REQUIRED" "" "" ${ARGN})
  SET(BOOST_PYTHON_NAME "python")
  SET(BOOST_PYTHON_REQUIRED "")
  IF(_BOOST_PYTHON_REQUIRED)
    SET(BOOST_PYTHON_REQUIRED REQUIRED)
  ENDIF(_BOOST_PYTHON_REQUIRED)

  SET_BOOST_DEFAULT_OPTIONS()

  IF(NOT PYTHONLIBS_FOUND)
    MESSAGE(FATAL_ERROR "Python has not been found. You should first call FindPython before calling SEARCH_FOR_BOOST_PYTHON macro.")
  ENDIF(NOT PYTHONLIBS_FOUND)

  # Test: pythonX, pythonXY and python-pyXY
  SET(BOOST_PYTHON_COMPONENT_LIST
    "python${PYTHON_VERSION_MAJOR}"
    "python${PYTHON_VERSION_MAJOR}${PYTHON_VERSION_MINOR}"
    "python-py${PYTHON_VERSION_MAJOR}${PYTHON_VERSION_MINOR}")

  SET(BOOST_PYTHON_FOUND FALSE)
  FOREACH(BOOST_PYTHON_COMPONENT ${BOOST_PYTHON_COMPONENT_LIST})
    SEARCH_FOR_BOOST_COMPONENT(${BOOST_PYTHON_COMPONENT} BOOST_PYTHON_FOUND)
    IF(BOOST_PYTHON_FOUND)
      SET(BOOST_PYTHON_NAME ${BOOST_PYTHON_COMPONENT})
      BREAK()
    ENDIF(BOOST_PYTHON_FOUND)
  ENDFOREACH(BOOST_PYTHON_COMPONENT ${BOOST_PYTHON_COMPONENT_LIST})

  # If boost-python has not been found, warn the user, and look for just "python"
  IF(NOT BOOST_PYTHON_FOUND)
    MESSAGE(WARNING "Impossible to check Boost.Python version. Trying with 'python'.")
  ENDIF(NOT BOOST_PYTHON_FOUND)

  FIND_PACKAGE(Boost ${BOOST_PYTHON_REQUIRED} COMPONENTS ${BOOST_PYTHON_NAME})
  STRING(TOUPPER ${BOOST_PYTHON_NAME} UPPERCOMPONENT)

  LIST(APPEND LOGGING_WATCHED_VARIABLES
    Boost_${UPPERCOMPONENT}_FOUND
    Boost_${UPPERCOMPONENT}_LIBRARY
    Boost_${UPPERCOMPONENT}_LIBRARY_DEBUG
    Boost_${UPPERCOMPONENT}_LIBRARY_RELEASE
    )

  SET(Boost_PYTHON_LIBRARY ${Boost_${UPPERCOMPONENT}_LIBRARY})
  MESSAGE(STATUS "Boost_PYTHON_LIBRARY: ${Boost_PYTHON_LIBRARY}")
  LIST(APPEND Boost_PYTHON_LIBRARIES ${Boost_PYTHON_LIBRARY})
  LIST(APPEND LOGGING_WATCHED_VARIABLES Boost_PYTHON_LIBRARY)
ENDMACRO(SEARCH_FOR_BOOST_PYTHON)

#.rst:
# .. command:: TARGET_LINK_BOOST_PYTHON (TARGET <PRIVATE|PUBLIC|INTERFACE>)
#
#   Link target againt boost_python library.
#
#   :target: is either a library or an executable
#   :private,public,interface: The PUBLIC, PRIVATE and INTERFACE keywords can be used to specify both the link dependencies and the link interface.
#
#   On darwin systems, boost_python is not linked against any python library.
#   This linkage is resolved at execution time via the python interpreter.
#   We then need to stipulate that boost_python has unresolved symbols at compile time for a library target.
#   Otherwise, for executables we need to link to a specific version of python.
#
MACRO(TARGET_LINK_BOOST_PYTHON target)
  IF(${ARGC} GREATER 1)
    SET(PUBLIC_KEYWORD ${ARGV1})
  ENDIF()

  IF(APPLE)
    GET_TARGET_PROPERTY(TARGET_TYPE ${target} TYPE)

    IF(${TARGET_TYPE} MATCHES EXECUTABLE)
      TARGET_LINK_LIBRARIES(${target} ${PUBLIC_KEYWORD} ${Boost_PYTHON_LIBRARY})
    ELSE(${TARGET_TYPE} MATCHES EXECUTABLE)
      TARGET_LINK_LIBRARIES(${target} ${PUBLIC_KEYWORD} -Wl,-undefined,dynamic_lookup,${Boost_PYTHON_LIBRARIES})
    ENDIF(${TARGET_TYPE} MATCHES EXECUTABLE)

  ELSE(APPLE)
    TARGET_LINK_LIBRARIES(${target} ${PUBLIC_KEYWORD} ${Boost_PYTHON_LIBRARY})
  ENDIF(APPLE)
  LIST(APPEND LOGGING_WATCHED_VARIABLES
    Boost_PYTHON_LIBRARIES
    )
ENDMACRO(TARGET_LINK_BOOST_PYTHON)
