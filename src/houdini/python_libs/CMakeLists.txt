set(DEPS_DIR "${EXTERNALS_INSTALL_DIR}")

include_directories( 
    ${EXTERNALS_INSTALL_DIR}
)

file( GLOB SOURCES
    ./piston.cpp
)

# Find python and Boost - both are required dependencies
if(NOT Houdini_FOUND)
    message( FATAL_ERROR "Houdini installation not found !" )
endif()

set(PYTHON_LIBS_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/houdini_${Houdini_VERSION}/python${_houdini_python_version}libs)

# Without this, any build libraries automatically have names "lib{x}.so"
set(CMAKE_SHARED_MODULE_PREFIX "_")

# Add a shared module - modules are intended to be imported at runtime.
# - This is where you add the source files
add_library(piston MODULE ${SOURCES} ${HEADERS})

# Set up the libraries and header search paths for this target
target_link_libraries( piston 
	Houdini
	piston_lib
)

houdini_configure_target( piston
	LIB_PREFIX ${CMAKE_SHARED_MODULE_PREFIX}
	INSTDIR ${PYTHON_LIBS_INSTALL_DIR}
)

# Make piston.py wrapper script
configure_file(piston.py.in ${CMAKE_CURRENT_BINARY_DIR}/piston.py.install @ONLY)

install(
    PROGRAMS
    	${CMAKE_CURRENT_BINARY_DIR}/piston.py.install
    RENAME
    	piston.py
    DESTINATION
    	${PYTHON_LIBS_INSTALL_DIR}
    PERMISSIONS 
    	OWNER_WRITE OWNER_READ
    	GROUP_READ OWNER_READ
        WORLD_READ
)