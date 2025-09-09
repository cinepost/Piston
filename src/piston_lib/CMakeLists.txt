include(${PROJECT_SOURCE_DIR}/third_party/external_versions.cmake)

include_directories( 
    ${EXTERNALS_INSTALL_DIR}
    ${CMAKE_THIRD_PARTY_SOURCE_DIR}/external/boost/src
    ${CMAKE_THIRD_PARTY_SOURCE_DIR}/bs_thread_pool
    ${CMAKE_THIRD_PARTY_SOURCE_DIR}/json/single_include
)

file( GLOB SOURCES
    ./common.cpp
    ./pxr_json.cpp
    ./adjacency.cpp
    ./curves_container.cpp
    ./phantom_trimesh.cpp
    ./geometry_tools.cpp
    ./serializable_data.cpp
    ./base_curves_deformer.cpp
    ./fast_curves_deformer.cpp
    ./fast_curves_deformer_data.cpp
    ./wrap_curves_deformer.cpp
    ./wrap_curves_deformer_data.cpp
    ./curves_deformer_factory.cpp
    ./simple_profiler.cpp
    ./deformer_stats.cpp
    ./tests.cpp
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

add_library( piston_lib STATIC ${SOURCES} ${HEADERS} )

if(WIN32)
#    set(USD_LIBS
#        hboost_python${HOUDINI_PYTHON_VERSION_STRIP}-mt-x64.lib
#        libgusd.lib
#        libpxr_hf.lib
#        libpxr_usdLux.lib
#        libpxr_hd.lib
#        libpxr_ar.lib
#        libpxr_arch.lib
#        libpxr_gf.lib
#        libpxr_js.lib
#        libpxr_kind.lib
#        libpxr_pcp.lib
#        libpxr_plug.lib
#        libpxr_sdf.lib
#        libpxr_tf.lib
#        libpxr_usd.lib
#        libpxr_usdGeom.lib
#        libpxr_usdRi.lib
#        libpxr_usdShade.lib
#        libpxr_usdUtils.lib
#        libpxr_vt.lib
#        libpxr_work.lib
#        libpxr_trace.lib
#    )

    set(_HOU_DSOLIB_ ${HROOT}/custom/houdini/dsolib)

    target_link_libraries(piston_lib 
        ${_HOU_DSOLIB_}/hboost_python${_houdini_python_dotless_version}-mt-x64.lib
        ${_HOU_DSOLIB_}/libgusd.lib
        ${_HOU_DSOLIB_}/libpxr_hf.lib
        ${_HOU_DSOLIB_}/libpxr_usdLux.lib
        ${_HOU_DSOLIB_}/libpxr_hd.lib
        ${_HOU_DSOLIB_}/libpxr_ar.lib
        ${_HOU_DSOLIB_}/libpxr_arch.lib
        ${_HOU_DSOLIB_}/libpxr_gf.lib
        ${_HOU_DSOLIB_}/libpxr_js.lib
        ${_HOU_DSOLIB_}/libpxr_kind.lib
        ${_HOU_DSOLIB_}/libpxr_pcp.lib
        ${_HOU_DSOLIB_}/libpxr_plug.lib
        ${_HOU_DSOLIB_}/libpxr_sdf.lib
        ${_HOU_DSOLIB_}/libpxr_tf.lib
        ${_HOU_DSOLIB_}/libpxr_usd.lib
        ${_HOU_DSOLIB_}/libpxr_usdGeom.lib
        ${_HOU_DSOLIB_}/libpxr_usdRi.lib
        ${_HOU_DSOLIB_}/libpxr_usdShade.lib
        ${_HOU_DSOLIB_}/libpxr_usdUtils.lib
        ${_HOU_DSOLIB_}/libpxr_vt.lib
        ${_HOU_DSOLIB_}/libpxr_work.lib
        ${_HOU_DSOLIB_}/libpxr_trace.lib
    )

endif() #WIN32

target_link_libraries( piston_lib
    Houdini
)
