include_directories( 
    ${HROOT}/toolkit/include
)

# Find python and Boost - both are required dependencies
if(NOT Houdini_FOUND)
    message( FATAL_ERROR "Houdini installation not found !" )
endif()

# Define the shared library plugin target
add_library(lop_deformer SHARED lop_deformer.cpp)

# Pixar USD explicit dependencies configuration
# Houdini embeds USD libraries into specific internal target names
target_compile_definitions(lop_deformer PRIVATE 
    -DPXR_USE_NAMESPACES=1
)
target_link_libraries(lop_deformer PRIVATE 
    Houdini
)

if(WIN32)

    target_compile_definitions(lop_deformer PRIVATE 
        WIN32
        _WINDOWS
        AMD64
        SIZEOF_VOID_P=8
        making_dso
        making_plugin
        SESI_LITTLE_ENDIAN
    )

    # Windows requires explicit symbol exports or empty prefix handling
    set_target_properties(lop_deformer PROPERTIES PREFIX "")
else()
    target_compile_definitions(lop_deformer PRIVATE 
        LINUX
        AMD64
        _GNU_SOURCE
        SIZEOF_VOID_P=${CMAKE_SIZEOF_VOID_P}
        making_dso
        making_plugin
        SESI_LITTLE_ENDIAN
    )

    # Unix specifications require position-independent compilation strings
    set_target_properties(lop_deformer PROPERTIES 
        POSITION_INDEPENDENT_CODE ON
        CXX_VISIBILITY_PRESET hidden
        VISIBILITY_INLINES_HIDDEN ON
    )

    target_link_libraries(lop_deformer PRIVATE pthread)
endif()


install(TARGETS lop_deformer DESTINATION "${HOUDINI_INSTALL_DIR}/dso")
