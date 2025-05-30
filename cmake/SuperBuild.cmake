include (ExternalProject)

message("Running SuperBuild.cmake")

list(APPEND 3RD_ARGS "-DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}")

if( DEFINED DEPS_BUILD_TYPE)
  list(APPEND 3RD_ARGS "-DDEPS_BUILD_TYPE=${DEPS_BUILD_TYPE}")
else()
  set(DEPS_BUILD_TYPE ${CMAKE_BUILD_TYPE})
  list(APPEND 3RD_ARGS "-DDEPS_BUILD_TYPE=${CMAKE_BUILD_TYPE}")
endif()

list(APPEND 3RD_ARGS "-DPISTON_3RD_LIB_LINK_WITH_STATIC:BOOL=OFF")
list(APPEND SRC_ARGS "-DBUILD_3RD_MEGALIB:BOOL=OFF")

set(EXTERNALS_INSTALL_DIR ${PROJECT_SOURCE_DIR}/deps/${DEPS_BUILD_TYPE})

message("Externals build type: " ${DEPS_BUILD_TYPE})

# Third party external projects
ExternalProject_Add( third_party
  SOURCE_DIR ${PROJECT_SOURCE_DIR}/third_party
  INSTALL_COMMAND ""
  ALWAYS ON
  BUILD_ALWAYS ON
  CONFIGURE_HANDLED_BY_BUILD ON
  BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/third_party
  CMAKE_ARGS ${3RD_ARGS} -DEXTERNALS_INSTALL_DIR=${EXTERNALS_INSTALL_DIR} -DDEPS_BUILD_TYPE=${DEPS_BUILD_TYPE}
)

# Main project
ExternalProject_Add( ep_src
  DEPENDS third_party
  SOURCE_DIR ${PROJECT_SOURCE_DIR}
  CMAKE_ARGS -DUSE_SUPERBUILD=OFF ${SRC_ARGS} -DEXTERNALS_INSTALL_DIR=${EXTERNALS_INSTALL_DIR}
  INSTALL_COMMAND ""
  BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}
)
add_dependencies(ep_src third_party)

add_custom_target(dummy COMMAND echo "Dummy main project target")
add_dependencies(dummy 
  third_party
  ep_src
)
