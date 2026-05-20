set(HOUDINI_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/houdini_${Houdini_VERSION})

add_subdirectory( python_libs )
#add_subdirectory( lops )

install(DIRECTORY toolbar DESTINATION ${HOUDINI_INSTALL_DIR})
install(DIRECTORY config DESTINATION ${HOUDINI_INSTALL_DIR})
