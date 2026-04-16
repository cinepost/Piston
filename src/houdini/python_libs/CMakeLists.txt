set(DEPS_DIR "${EXTERNALS_INSTALL_DIR}")

include_directories( 
    ${EXTERNALS_INSTALL_DIR}
    ${CMAKE_THIRD_PARTY_SOURCE_DIR}/bs_thread_pool
    ${CMAKE_THIRD_PARTY_SOURCE_DIR}/json/single_include
)

file( GLOB SOURCES
    ./piston.generated.cpp
)

# Find python and Boost - both are required dependencies
if(NOT Houdini_FOUND)
    message( FATAL_ERROR "Houdini installation not found !" )
endif()

set(HOUDINI_PYTHON_LIBS_INSTALL_DIR ${HOUDINI_INSTALL_DIR}/python${_houdini_python_version}libs)

# Without this, any build libraries automatically have names "lib{x}.so"
set(CMAKE_SHARED_MODULE_PREFIX "_")

# Generates boost python docstrings

find_package(Python3 REQUIRED COMPONENTS Interpreter)

add_custom_command(
    OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/piston.generated.cpp"
    COMMAND Python3::Interpreter ${PROJECT_SOURCE_DIR}/tools/boost_python_docstrings.py
            --srcdir ${PROJECT_SOURCE_DIR}/src/piston_lib
            --template ${CMAKE_CURRENT_SOURCE_DIR}/piston.cpp
            --output ${CMAKE_CURRENT_BINARY_DIR}/piston.generated.cpp
    DEPENDS ${PROJECT_SOURCE_DIR}/tools/boost_python_docstrings.py
            ${CMAKE_CURRENT_SOURCE_DIR}/piston.cpp
    COMMENT "Running Python script to configure boost python module C++ source"
    VERBATIM
)

# Add a shared module - modules are intended to be imported at runtime.
# - This is where you add the source files
add_library(piston MODULE ${CMAKE_CURRENT_BINARY_DIR}/piston.generated.cpp ${HEADERS})

# add_dependencies(piston run)

# Set up the libraries and header search paths for this target
target_link_libraries( piston 
	Houdini
	piston_lib
)

if(WIN32)
target_link_libraries( piston 
   ${HROOT}/python${_houdini_python_dotless_version}/libs/python${_houdini_python_dotless_version}.lib
)
endif()

houdini_configure_target( piston
	LIB_PREFIX ${CMAKE_SHARED_MODULE_PREFIX}
	INSTDIR ${HOUDINI_PYTHON_LIBS_INSTALL_DIR}
)

if(WIN32)
    set_target_properties(piston PROPERTIES SUFFIX ".pyd")
endif()

# Make piston.py wrapper script
configure_file(piston.py.in ${CMAKE_CURRENT_BINARY_DIR}/piston.py.install @ONLY)

install(
    PROGRAMS
    	${CMAKE_CURRENT_BINARY_DIR}/piston.py.install
    RENAME
    	piston.py
    DESTINATION
    	${HOUDINI_PYTHON_LIBS_INSTALL_DIR}
    PERMISSIONS 
    	OWNER_WRITE OWNER_READ
    	GROUP_READ OWNER_READ
        WORLD_READ
)

install(FILES pythonrc.py DESTINATION ${HOUDINI_PYTHON_LIBS_INSTALL_DIR})
install(DIRECTORY piston_houdini DESTINATION ${HOUDINI_PYTHON_LIBS_INSTALL_DIR})