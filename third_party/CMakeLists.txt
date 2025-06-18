cmake_minimum_required(VERSION 3.20)

project("ThirdParty_External")

include(FindPkgConfig)
include(ExternalProject)
include(ProcessorCount)

message("Running third party dependencies build...")

#we need git executable
find_package(Git REQUIRED)

set( CMAKE_MODULE_PATH 
  ${PROJECT_SOURCE_DIR}/../cmake/modules
)

include(${PROJECT_SOURCE_DIR}/external_versions.cmake)

set(EXTRA_CMAKE_ARGS)

set(EXTERNALS_BINARY_DIR ${PROJECT_SOURCE_DIR}/../build/${DEPS_BUILD_TYPE}/external)
set(EXTERNALS_STAMP_DIR ${PROJECT_SOURCE_DIR}/../build/${DEPS_BUILD_TYPE}/external/stamp)
set(EXTERNALS_TMP_DIR ${PROJECT_SOURCE_DIR}/../build/${DEPS_BUILD_TYPE}/external/tmp)

set(EXTERNALS_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external)

find_program(MAKE_EXECUTABLE
  NAMES gmake mingw32-make make
  NAMES_PER_DIR
  DOC "GNU Make"
)

if(WIN32)
  set(MAKE_EXECUTABLE cmake --build . -j 8 --config ${DEPS_BUILD_TYPE})
endif()

find_program(CHMOD_EXECUTABLE
  NAMES chmod
  NAMES_PER_DIR
)

find_package(Git)
if(Git_FOUND)
  message("Git found: ${GIT_EXECUTABLE}")
endif()


#######################
# Boost
#######################

set(Boost_USE_MULTITHREADED ON)

if(PISTON_3RD_LIB_LINK_WITH_STATIC)
  set(Boost_USE_STATIC_LIBS ON)
  set(Boost_USE_STATIC_RUNTIME ON)
endif()

if( NOT EXTERNAL_BOOST_VERSION )
  set(EXTERNAL_BOOST_VERSION 1.82.0)
endif()

set(BOOST_ROOT ${EXTERNALS_INSTALL_DIR})
find_package(Boost ${EXTERNAL_BOOST_VERSION} COMPONENTS python system date_time iostreams EXACT)

if(Boost_FOUND)

  message("Boost version ${Boost_VERSION} found ...")
  message(Boost_INCLUDE_DIR "${Boost_INCLUDE_DIR}")
  add_custom_target( Boost COMMAND echo "Boost ${EXTERNAL_BOOST_VERSION} build exists, skipping rebuild ...")

else()

  message("Boost version ${EXTERNAL_BOOST_VERSION} not found! Running external build ...")
  set(BOOST_PREFIX boost)

  if(LAVA_3RD_LIB_LINK_WITH_STATIC)
    set(BOOST_LINK_STYLE static)
    set(BOOST_CXX_FLAGS "cxxflags=-fPIC")
    set(BOOST_C_FLAGS "cflags=-fPIC")
  else()
    set(BOOST_LINK_STYLE shared)
    set(BOOST_CXX_FLAGS "")
    set(BOOST_C_FLAGS "")
  endif()

  if(DEPS_BUILD_TYPE STREQUAL "Debug")
    set(BOOST_BUILD_TYPE "debug")
  else()
    set(BOOST_BUILD_TYPE "release")
  endif()

  if(WIN32)
    ExternalProject_Add ( Boost
      URL       https://archives.boost.io/release/1.82.0/source/boost_1_82_0.zip
      URL_HASH  SHA256=f7c9e28d242abcd7a2c1b962039fcdd463ca149d1883c3a950bbcc0ce6f7c6d9
      CONFIGURE_COMMAND ""
      BUILD_COMMAND ""
      BUILD_IN_SOURCE true
      INSTALL_COMMAND ""
      INSTALL_DIR ${EXTERNALS_INSTALL_DIR}
      TMP_DIR ${EXTERNALS_TMP_DIR}/boost
      DOWNLOAD_DIR ${EXTERNALS_SOURCE_DIR}/${BOOST_PREFIX}
      SOURCE_DIR ${EXTERNALS_SOURCE_DIR}/${BOOST_PREFIX}/src
      STAMP_DIR ${EXTERNALS_STAMP_DIR}/boost
    )
  else()
    ExternalProject_Add ( Boost
      URL       https://archives.boost.io/release/1.82.0/source/boost_1_82_0.tar.gz
      URL_HASH  SHA256=66a469b6e608a51f8347236f4912e27dc5c60c60d7d53ae9bfe4683316c6f04c
      CONFIGURE_COMMAND ""
      BUILD_COMMAND ""
      BUILD_IN_SOURCE true
      INSTALL_COMMAND ""
      INSTALL_DIR ${EXTERNALS_INSTALL_DIR}
      TMP_DIR ${EXTERNALS_TMP_DIR}/boost
      DOWNLOAD_DIR ${EXTERNALS_SOURCE_DIR}/${BOOST_PREFIX}
      SOURCE_DIR ${EXTERNALS_SOURCE_DIR}/${BOOST_PREFIX}/src
      STAMP_DIR ${EXTERNALS_STAMP_DIR}/boost
    )
  endif()
endif()

#######################

add_custom_target( dummy_target COMMAND echo "Third party project dummy target")
add_dependencies( dummy_target
  Boost
)
