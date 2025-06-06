include(${PROJECT_SOURCE_DIR}/third_party/external_versions.cmake)

include_directories( 
    ${EXTERNALS_INSTALL_DIR}
)

file( GLOB SOURCES
    ./common.cpp
    ./adjacency.cpp
    ./curves_container.cpp
    ./phantom_trimesh.cpp
    ./geometry_tools.cpp
    ./base_curves_deformer.cpp
    ./fast_curves_deformer.cpp
    ./curves_deformer_factory.cpp
)

# Find python and Boost - both are required dependencies
if(NOT Houdini_FOUND)
    message( FATAL_ERROR "Houdini installation not found !" )
endif()

if(WIN32)
    add_definitions(-DAMD64 -DSIZEOF_VOID_P=${CMAKE_SIZEOF_VOID_P} -DLAVA_DLL)
else()
    add_definitions(-DAMD64 -DLINUX -DUSE_PTHREADS -DSIZEOF_VOID_P=${CMAKE_SIZEOF_VOID_P})
endif()

# Without this, any build libraries automatically have names "lib{x}.so"
set(CMAKE_SHARED_MODULE_PREFIX "")

add_library( piston_lib SHARED ${SOURCES} ${HEADERS} )

target_link_libraries( piston_lib
    Houdini
)

houdini_configure_target( piston_lib
    LIB_PREFIX ${CMAKE_SHARED_MODULE_PREFIX}
    INSTDIR ${CMAKE_INSTALL_PREFIX}/lib
)

#if(UNIX OR WIN32)
#    install(TARGETS piston_lib
#        RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
#        LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
#        ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
#    )
#endif()
