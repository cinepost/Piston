cmake_minimum_required( VERSION 3.21 )

if(UNIX)
set(CMAKE_C_COMPILER gcc-11)
set(CMAKE_CXX_COMPILER g++-11)
endif()

set( CMAKE_CXX_STANDARD 17 )
set( CMAKE_CXX_STANDARD_REQUIRED ON )
set( CMAKE_CXX_EXTENSIONS ON )

set( PROJECT_VERSION_MAJOR 0 )
set( PROJECT_VERSION_MINOR 0 )
set( PROJECT_VERSION_PATCH 001 )

#### Handle build mode
# Set a default build type if none was specified
set(default_build_type "Release")
if(EXISTS "${CMAKE_SOURCE_DIR}/.git")
  set(default_build_type "Debug")
endif()

if( NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES )
  message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
  set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE STRING "Choose the type of build." FORCE)

  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

if(UNIX)
  set(DEFAULT_HFS_PATH "/opt/hfs20.5")
  set(DEFAULT_HIH_PATH "$ENV{HOME}/houdini20.5")
else() # WIN
  set(USER_DOCUMENTS_PATH "$ENV{USERPROFILE}/Documents")
  set(DEFAULT_HFS_PATH "C:\\Program Files\\Side Effects Software\\Houdini 20.5.332")
  set(DEFAULT_HIH_PATH "${USER_DOCUMENTS_PATH}\\houdini20.5")
endif()

if (NOT DEFINED ENV{HFS})
  message( WARNING "Houdini HFS environment variable not set. Using default path: ${DEFAULT_HFS_PATH}")
  set(ENV{HFS} ${DEFAULT_HFS_PATH})
endif()

if (NOT DEFINED ENV{HIH})
  message( WARNING "Houdini HIH environment variable not set. Using defalut path: ${DEFAULT_HIH_PATH}")
  set(ENV{HIH} ${DEFAULT_HIH_PATH})
endif()

set(HROOT $ENV{HFS})
message( "Using Houdini installation path: " ${HROOT}/toolkit/cmake)
list( APPEND CMAKE_PREFIX_PATH "${HROOT}/toolkit/cmake" )

if(WIN32)
  set(ENV{LIBPATH} "$ENV{LIBPATH};${HROOT}\\python311\\libs;${HROOT}")
  set(ENV{LIB} "$ENV{LIB};${HROOT}\\python311\\libs;${HROOT}\\custom\\houdini\\dsolib")
endif() #WIN32

# Install path
if(CMAKE_INSTALL_PREFIX)
  set(linuxDefault ${CMAKE_INSTALL_PREFIX} STREQUAL "/usr/local")
  set(windowsDefault ${CMAKE_INSTALL_PREFIX} STREQUAL "C:\\Program Files (x86)")
else()
  set(linuxDefault TRUE)
  set(windowsDefault TRUE)
endif()

if(WIN32)
  set( DEFAULT_CMAKE_INSTALL_PREFIX "D:/work/piston")
else()
  set( DEFAULT_CMAKE_INSTALL_PREFIX "/opt/piston" )
endif()

# Override default install path
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT OR ${linuxDefault} OR ${windowsDefault})
  set( CMAKE_INSTALL_PREFIX ${DEFAULT_CMAKE_INSTALL_PREFIX} CACHE PATH "Piston install prefix" FORCE)
endif()

message("Install path: ${CMAKE_INSTALL_PREFIX}")

# Handle superbuild first
option (USE_SUPERBUILD "Whether or not a superbuild should be invoked" OFF)

if(USE_SUPERBUILD)
  project (SUPERBUILD NONE)
  # execute the superbuild (this script will be invoked again without the USE_SUPERBUILD option this time)
  include (cmake/SuperBuild.cmake)
  return()
else()
  # Basic information about project
  set( PROJECT_VERSION ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH} )
  project( Piston VERSION ${PROJECT_VERSION} )
endif()

# Version generators
#include (cmake/GenerateLavaVersionCpp.cmake)
#include (cmake/GenerateLavaVersionPython.cmake)

# ccache
#include(cmake/CPM.cmake)

# Add folder where are supportive functions
set( CMAKE_MODULE_PATH
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake 
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/defaults
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/macros
)
set( CMAKE_INCLUDE_CURRENT_DIR ON )

# Set additional project information
set( COMPANY "Parovoz" )
set( COPYRIGHT "Copyright (c) Parovoz" )
set( IDENTIFIER "com.parovoz" )

# Set python virtualenv for USD
if(UNIX)
set( PYTHON_EXECUTABLE "python3.11")
endif()

# Specify build paths
set( CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/${CMAKE_BUILD_TYPE}/bin)
set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_BUILD_TYPE}/lib )
set( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_BUILD_TYPE}/lib )
set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_BUILD_TYPE}/bin )

add_definitions(-DPISTON_INSTALL_DIR="${CMAKE_INSTALL_PREFIX}")

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_definitions(-D_DEBUG)
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DDEBUG")
endif ()

if(WIN32)
add_compile_definitions(
  $<$<CONFIG:RELEASE>:NDEBUG>
  $<$<CONFIG:DEBUG>:_DEBUG>
  $<$<CONFIG:DEBUG>:Py_DEBUG>
  )
endif()

if ( MSVC )
  ADD_DEFINITIONS(/W3 /std:c++17 /permissive- /MP /wd4005 /wd4068)
  add_compile_options($<$<CONFIG:DEBUG>:/bigobj>)
elseif ( UNIX ) #(CMAKE_COMPILER_IS_GNUCXX)

  set(CMAKE_CXX_FLAGS "-pthread -Wall -Wextra")
  
  set(CMAKE_C_FLAGS_DEBUG "-std=c++17 -pthread -g -O0")
  set(CMAKE_CXX_FLAGS_DEBUG "-std=c++17 -pthread -g -O0")

  set(CMAKE_C_FLAGS_RELEASE "-O3")
  set(CMAKE_CXX_FLAGS_RELEASE "-O3")


  ADD_DEFINITIONS(-std=c++17 -pthread -Wall -fPIC -Wfatal-errors) # -Wno-reorder -pedantic
else ()
  MESSAGE(FATAL_ERROR "CMakeLists.txt has not been tested/written for your compiler.")
endif ()

set( PISTON_INCLUDE_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src )
set( CMAKE_THIRD_PARTY_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party)
include_directories(
  ${PISTON_INCLUDE_ROOT_DIR}
  ${PISTON_INCLUDE_ROOT_DIR}/Piston
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party
)

# use, i.e. don't skip the full RPATH for the build tree
set(CMAKE_SKIP_BUILD_RPATH FALSE)

# when building, don't use the install RPATH already
# (but later on when installing)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)

set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib;${CMAKE_INSTALL_PREFIX}/lib/3rdparty")

# add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

#------------------------------------------
# Our stuff
#------------------------------------------
find_package( Houdini REQUIRED )
message("Houdini python version: ${_houdini_python_version}")

if(WIN32)
  set(ENV{LIBPATH} "$ENV{LIBPATH};${HROOT}\\python${_houdini_python_dotless_version}\\libs;${HROOT}")
  set(ENV{LIB} "$ENV{LIB};${HROOT}\\python${_houdini_python_dotless_version}\\libs;${HROOT}\\custom\\houdini\\dsolib")
endif() #WIN32

add_subdirectory( src )
#add_subdirectory( contrib )

#install(FILES src/changelog.txt DESTINATION ${CMAKE_INSTALL_PREFIX})

#------------------------------------------
# Install third party dependencies
#------------------------------------------

# third party libs
if(WIN32)
  install(DIRECTORY ${EXTERNALS_INSTALL_DIR}/lib/ DESTINATION lib/3rdparty
    FILES_MATCHING PATTERN "*.dll"
    PATTERN "*.lib"
    PATTERN "cmake" EXCLUDE
    PATTERN "pkgconfig" EXCLUDE
    PATTERN "libpng" EXCLUDE
  )
else()
  install(DIRECTORY ${EXTERNALS_INSTALL_DIR}/lib/ DESTINATION lib/3rdparty
    FILES_MATCHING PATTERN "*.so*"
    PATTERN "cmake" EXCLUDE
    PATTERN "pkgconfig" EXCLUDE
    PATTERN "libpng" EXCLUDE
  )
endif()
