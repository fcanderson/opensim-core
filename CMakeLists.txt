project(OpenSim)
#
# Build of OpenSim. There are three steps:
#    (1) Choose appropriate platform
#    (2) Locate Simbody and its dependent libraries
#    (3) Build OpenSim libraries and executables
#
#
#----------------------------------------------------

cmake_minimum_required(VERSION 2.8.8)
if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
    cmake_policy(SET CMP0005 NEW)
    if(NOT (${CMAKE_VERSION} VERSION_LESS 3.0))
        # MACOSX_RPATH enabled by default; policy introduced with cmake 3.0.
        cmake_policy(SET CMP0042 NEW)
    endif()
endif(COMMAND cmake_policy)


# OpenSim version.
# ----------------
set(OPENSIM_MAJOR_VERSION 4)
set(OPENSIM_MINOR_VERSION 0)
set(OPENSIM_PATCH_VERSION 0)

# Don't include the patch version if it is 0.
set(PATCH_VERSION_STRING)
if(OPENSIM_PATCH_VERSION)
    set(PATCH_VERSION_STRING ".${OPENSIM_PATCH_VERSION}")
endif()

set(OPENSIM_VERSION
    "${OPENSIM_MAJOR_VERSION}.${OPENSIM_MINOR_VERSION}${PATCH_VERSION_STRING}"
    )


# CMake module path.
# ------------------
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
include(OpenSimMacros)


# Build binary directory.
# -----------------------
set(BUILD_BINARY_DIR ${CMAKE_BINARY_DIR}
    CACHE PATH
    "The OpenSim build (not the install) puts all the libraries and executables together here (with /Release, etc. appended on some platforms).")

# Make everything go in the same binary directory. (These are CMake-defined
# variables.)
set(EXECUTABLE_OUTPUT_PATH ${BUILD_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${BUILD_BINARY_DIR})
link_directories(${BUILD_BINARY_DIR})


# Specify number of cores to run for build
set(PROCESSOR_COUNT 8 CACHE STRING "Number of processors in the CPU")
mark_as_advanced(PROCESSOR_COUNT)


# Simbody.
# --------
# This will be initialized to the environment variable of the same name
# if it is set, otherwise it will be empty.
set(SIMBODY_HOME $ENV{SIMBODY_HOME} CACHE
    PATH "The location of the Simbody installation to use; you can change this. Set as empty to let CMake search for Simbody automatically.")

option(OPENSIM_COPY_SIMBODY "Copy Simbody headers and libraries into the
OpenSim installation. This way, you can treat OpenSim and Simbody as one
package, and you only need to set environment variables (e.g., PATH,
LD_LIBRARY_PATH, DYLD_LIBRARY_PATH) for OpenSim.  On Windows, this also copies
Simbody dll's and exe's into the OpenSim build directory, so that you don't
need to set PATH to run OpenSim tests and examples; this is done via the
SimbodyFiles project. If OFF, you likely must set those environment variables
for both OpenSim and Simbody." ON)


# Platform.
# ---------
# Create a platform name useful for finding things in the Platform
# directory.
if(WIN32)
    if(CMAKE_GENERATOR MATCHES "Visual Studio 10")
        #NOTE: VC12 (VS2013) does not require explicit link line to Psapi library
        #Specifying here so it will link for VC10
        link_libraries(Psapi)
        set(VCVERSION VC10)
    endif()
    set(Platform "Windows")
    set(NATIVE_COPY_CMD copy)
elseif(APPLE)
    set(Platform Mac)
    set(PLATFORM_NAME Mac)
    set(NATIVE_COPY_CMD cp)
else()
    set(Platform Linux)
    set(PLATFORM_NAME Linux)
    set(NATIVE_COPY_CMD cp)
endif()

# In addition to the platform name we need to know the Application Binary
# Interface (ABI) we're building for. Currently that is either x86, meaning
# 32 bit Intel instruction set, or x64 for 64 bit Intel instruction set.
# Kevin: Since Ubuntu 12 64bit libraries are in lib not lib64 (This in
# response of Sherm's change on Simbody)

if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(PLATFORM_ABI x64)
    set(ARCH64 ON)
    #    set(LIB64 64) Ubuntu 12.05
else()
    set(PLATFORM_ABI x86)
endif()

set(BUILD_PLATFORM "${PLATFORM_NAME}:${PLATFORM_ABI}" CACHE STRING
    "This is the platform and ABI we're building for. Not changeable here; use a different CMake generator instead."
    FORCE)

if(NOT MSVC AND NOT XCODE AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Debug or Release build" FORCE)
endif()

## Choose the maximum level of x86 instruction set that the compiler is
## allowed to use. Was using sse2 but changed to let the compilers choose. Most
## will probably use sse2 or later by default.
## On 64 bit MSVC, the default is sse2 and the argument
## isn't recognized so we won't specify it.

if(CMAKE_CL_64)
    set(default_build_inst_set)
else()
    # Here's where we used to set sse2. Leaving this line in
    # case we decide to specify a default value again.
    set(default_build_inst_set)
endif()

## This can be set to a different value by the person running CMake.
set(BUILD_INST_SET ""
    CACHE STRING
    "CPU instruction level compiler is permitted to use (default: let compiler decide).")
mark_as_advanced( BUILD_INST_SET )

if(BUILD_INST_SET)
    set(inst_set_to_use ${BUILD_INST_SET})
else()
    set(inst_set_to_use ${default_build_inst_set})
endif()

## When building in any of the Release modes, tell gcc to use not-quite most
## aggressive optimization and to generate SSE2 floating point instructions.
## Here we are specifying *all* of the Release flags, overriding CMake's
## defaults.  Watch out for optimizer bugs in particular gcc versions!

if(${CMAKE_CXX_COMPILER_ID} MATCHES "GNU" OR
        ${CMAKE_CXX_COMPILER_ID} MATCHES "Clang")
    if(inst_set_to_use)
        string(TOLOWER ${inst_set_to_use} GCC_INST_SET)
        set(GCC_INST_SET "-m${GCC_INST_SET}")
    else()
        set(GCC_INST_SET)
    endif()

    # Get the gcc version number in major.minor.build format
    execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion
                    OUTPUT_VARIABLE GCC_VERSION)


    # Testing with Clang 3.3 on Ubuntu 14.04 shows a 5% decrease
    # in the runtime of the tests when we enable loop unrolling.
    set(GCC_OPT_ENABLE "-funroll-loops")

    # If you know of optimization bugs that affect SimTK in particular
    # gcc versions, this is the place to turn off those optimizations.
    set(GCC_OPT_DISABLE)
    # We know Gcc 4.4.3 on Ubuntu 10 is buggy and that Snow Leopard's
    # 4.2.1 is not. To be safe for now we'll assume anything over 4.3
    # should have these disabled.
    if(GCC_VERSION VERSION_GREATER 4.3 OR GCC_VERSION VERSION_EQUAL 4.3)
        set(GCC_OPT_DISABLE
    "-fno-tree-vrp -fno-strict-aliasing -fno-guess-branch-probability")
    endif()

    # C++
    set(CMAKE_CXX_FLAGS_DEBUG          "-g ${GCC_INST_SET}"
      CACHE STRING "g++ Debug build compile flags" FORCE)
    set(CMAKE_CXX_FLAGS_RELEASE
      "-DNDEBUG -O2 ${GCC_OPT_ENABLE} ${GCC_OPT_DISABLE} ${GCC_INST_SET}"
      CACHE STRING "g++ Release build compile flags" FORCE)
    set(CMAKE_CXX_FLAGS_RELWITHDEBINFO
      "-DNDEBUG -O2 -g ${GCC_OPT_ENABLE} ${GCC_OPT_DISABLE} ${GCC_INST_SET}"
      CACHE STRING "g++ RelWithDebInfo build compile flags" FORCE)
    set(CMAKE_CXX_FLAGS_MINSIZEREL     "-DNDEBUG -Os ${GCC_INST_SET}"
      CACHE STRING "g++ MinSizeRel build compile flags" FORCE)

    # C
    set(CMAKE_C_FLAGS_DEBUG            "-g ${GCC_INST_SET}"
      CACHE STRING "gcc Debug build compile flags" FORCE)
    set(CMAKE_C_FLAGS_RELEASE
      "-DNDEBUG -O2 ${GCC_OPT_ENABLE} ${GCC_OPT_DISABLE} ${GCC_INST_SET}"
      CACHE STRING "gcc Release build compile flags" FORCE)
    set(CMAKE_C_FLAGS_RELWITHDEBINFO
      "-DNDEBUG -O2 -g ${GCC_OPT_ENABLE} ${GCC_OPT_DISABLE} ${GCC_INST_SET}"
      CACHE STRING "gcc RelWithDebInfo build compile flags" FORCE)
    set(CMAKE_C_FLAGS_MINSIZEREL       "-DNDEBUG -Os ${GCC_INST_SET}"
      CACHE STRING "gcc MinSizeRel build compile flags" FORCE)

endif()

## When building in any of the Release modes, tell VC++ cl compiler to use intrinsics
## (i.e. sqrt instruction rather than sqrt subroutine) with flag /Oi.

if(WIN32 AND ${CMAKE_C_COMPILER} MATCHES "cl")
    if(inst_set_to_use)
        string(TOUPPER ${inst_set_to_use} CL_INST_SET)
        set(CL_INST_SET "/arch:${CL_INST_SET}")
    else()
        set(CL_INST_SET)
    endif()

    set(BUILD_LIMIT_PARALLEL_COMPILES "" CACHE STRING
        "Set a maximum number of simultaneous compilations.")
    mark_as_advanced(BUILD_LIMIT_PARALLEL_COMPILES)
    set(mxcpu ${BUILD_LIMIT_PARALLEL_COMPILES}) # abbreviation

    ## C++
    set(CMAKE_CXX_FLAGS_DEBUG
    "/MP${mxcpu} /D _DEBUG /MDd /Od /Ob0 /RTC1 /Zi /bigobj /GS- ${CL_INST_SET}"
        CACHE STRING "VC++ Debug build compile flags" FORCE)
    set(CMAKE_CXX_FLAGS_RELEASE
    "/MP${mxcpu} /D NDEBUG /MD  /O2 /Ob2 /Oi /bigobj /GS- ${CL_INST_SET}"
        CACHE STRING "VC++ Release build compile flags" FORCE)
    set(CMAKE_CXX_FLAGS_RELWITHDEBINFO
    "/MP${mxcpu} /D NDEBUG /MD  /O2 /Ob2 /Oi /Zi /bigobj /GS- ${CL_INST_SET}"
        CACHE STRING "VC++ RelWithDebInfo build compile flags" FORCE)
    set(CMAKE_CXX_FLAGS_MINSIZEREL
    "/MP${mxcpu} /D NDEBUG /MD  /O1 /Ob1 /Oi /bigobj /GS- ${CL_INST_SET}"
        CACHE STRING "VC++ MinSizeRel build compile flags" FORCE)

    ## C
    set(CMAKE_C_FLAGS_DEBUG
    "/MP${mxcpu} /D _DEBUG /MDd /Od /Ob0 /RTC1 /Zi /GS- ${CL_INST_SET}"
        CACHE STRING "VC++ Debug build compile flags" FORCE)
    set(CMAKE_C_FLAGS_RELEASE
    "/MP${mxcpu} /D NDEBUG /MD  /O2 /Ob2 /Oi /GS- ${CL_INST_SET}"
        CACHE STRING "VC++ Release build compile flags" FORCE)
    set(CMAKE_C_FLAGS_RELWITHDEBINFO
    "/MP${mxcpu} /D NDEBUG /MD  /O2 /Ob2 /Oi /Zi /GS- ${CL_INST_SET}"
        CACHE STRING "VC++ RelWithDebInfo build compile flags" FORCE)
    set(CMAKE_C_FLAGS_MINSIZEREL
    "/MP${mxcpu} /D NDEBUG /MD  /O1 /Ob1 /Oi /GS- ${CL_INST_SET}"
        CACHE STRING "VC++ MinSizeRel build compile flags" FORCE)

endif()

set(BUILD_JAVA_WRAPPING OFF CACHE BOOL "Build Java wrapping (needed if you're building the GUI and have SWIG and Java installed on your machine.)")
mark_as_advanced( BUILD_JAVA_WRAPPING )

set(BUILD_PYTHON_WRAPPING OFF CACHE BOOL "Build Python wrapping (needed if you're building the Python wrapping and have SWIG and Python installed on your machine.)")
mark_as_advanced( BUILD_PYTHON_WRAPPING )

# You can enable this advanced-variable feature to cause Visual Studio
# or Makefiles to print the compile/link commands they are executing. Very
# useful for debugging CMake stuff but messy and obscures errors and warnings
# so should be off by default.
set(CMAKE_VERBOSE_MAKEFILE OFF CACHE BOOL
    "Enable this for verbose build output.")

if(WIN32)
add_definitions(-D_CRT_SECURE_NO_DEPRECATE)
else(WIN32)
endif(WIN32)

##################################################
## Define Platform to
##          Win32/VC${version}
##      or  Mac
##      or  Linux
##
## Also translate ARCH64 to platform specific flags
##
##################################################
include(InstallRequiredSystemLibraries)

if(WIN32)
else(WIN32)
    set(NameSpace "")   ## No renamed SimTK libraries except on Windows
    if(APPLE)
       if(ARCH64)
           set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmacosx-version-min=10.8" )
       else(ARCH64)
           set( CMAKE_CXX_FLAGS
               "${CMAKE_CXX_FLAGS} -mmacosx-version-min=10.8 -m32" )
       endif(ARCH64)
    else(APPLE)
       if(ARCH64)
           set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m64" )
       else(ARCH64)
          # I don't believe setting the CXX flags is working in cmake.
          set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m32" )
       endif(ARCH64)
    endif(APPLE)
endif(WIN32)

# C++11: In revision 8629, we started using std::unique_ptr, which requires
# C++11 features to be enabled when using GCC or Clang.
if(${CMAKE_C_COMPILER} MATCHES "cc" OR ${CMAKE_C_COMPILER} MATCHES "clang")
    # Using C++11 on OSX requires using libc++ instead of libstd++.
    # libc++ is an implementation of the C++ standard library for OSX.
    if(APPLE)
        if(XCODE)
            set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LANGUAGE_STANDARD "c++11")
            set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LIBRARY "libc++")
        else()
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
            if(${CMAKE_CXX_COMPILER_ID} MATCHES "Clang")
                set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
            endif()
        endif()
    else() # not APPLE
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    endif()
endif()


# Find Simbody.
# -------------
# As of Simbody 3.4, Simbody has a SimbodyConfig.cmake file, which is a
# preferred way to find Simbody over the previous FindSimbody.cmake script.
# NO_MODULE means we will not allow the use of a FindSimbody.cmake script.
set(SIMBODY_VERSION_TO_USE 3.5)

# Find Simbody freshly by unsetting this CMake-generated variable.
unset(Simbody_DIR CACHE)
if("${SIMBODY_HOME}" STREQUAL "")
    # We assume the only case in which the user
    # wants us to search for Simbody is if they left SIMBODY_HOME empty.
    # If the user specifies an invalid SIMBODY_HOME by accident,
    # we shouldn't let that fail silently and still search for
    # Simbody elsewhere; they may never realize
    # we are not using their requested installation of Simbody.
    find_package(Simbody ${SIMBODY_VERSION_TO_USE} QUIET
        NO_MODULE)
else()
    # Find the package using the user-specified path.
    # NO_DEFAULT_PATH will cause find_package to only
    # look in the provided PATHS.
    find_package(Simbody ${SIMBODY_VERSION_TO_USE} QUIET
        PATHS "${SIMBODY_HOME}" NO_MODULE NO_DEFAULT_PATH)
endif()
# This variable appears in the CMake GUI and could confuse users,
# since this variable can't be changed manually.
mark_as_advanced(Simbody_DIR)

# If Simbody is not found, Simbody_FOUND is false.
if(NOT Simbody_FOUND)
    message(FATAL_ERROR "
        Simbody ${SIMBODY_VERSION_TO_USE} not found. Install Simbody and set
        SIMBODY_HOME to the installation directory of Simbody.")
endif()

# Directories for Simbody headers and libraries for building.
# -----------------------------------------------------------
include_directories("${Simbody_INCLUDE_DIR}")
link_directories("${Simbody_LIB_DIR}")


# Copy files over from the Simbody installation.
# ----------------------------------------------
if(${OPENSIM_COPY_SIMBODY})

    # Install Simbody headers into OpenSim installation.
    install(DIRECTORY "${Simbody_INCLUDE_DIR}" DESTINATION sdk/include/SimTK)

    # Figure out which Simbody files need to be installed in the /bin
    # directory and which in the lib directory. Executables go in /bin on all
    # platforms.  Libraries (shared or otherwise) go in /lib, except on Windows
    # where the ".dll" files go in bin along with executables. Anything that
    # is only needed for programmers goes in sdk/lib.
    if(WIN32)
        file(GLOB SIMBODY_BIN_FILES
            ${Simbody_BIN_DIR}/*.dll
            ${Simbody_BIN_DIR}/*.exe)
        file(GLOB SIMBODY_SDKLIB_FILES
            ${Simbody_LIB_DIR}/*.lib)
    else()
        file(GLOB SIMBODY_BIN_FILES
            ${Simbody_BIN_DIR}/simbody*) # executables only (e.g., visualizer).
        # If the LIB_DIR is some common place for libraries (e.g., /usr/lib/),
        # we only want to copy over libraries that have SimTK in their name.
        file(GLOB SIMBODY_LIB_FILES
            ${Simbody_LIB_DIR}/*SimTK*.so
            ${Simbody_LIB_DIR}/*SimTK*.so.*
            ${Simbody_LIB_DIR}/*SimTK*.dylib)
        file(GLOB SIMBODY_SDKLIB_FILES
            ${Simbody_LIB_DIR}/*SimTK*.a)
    endif()

    # Specify "PROGRAMS" rather than "FILES" so the execute bit gets set.
    install(PROGRAMS ${SIMBODY_BIN_FILES} DESTINATION bin)
    install(FILES ${SIMBODY_LIB_FILES}    DESTINATION lib)
    install(FILES ${SIMBODY_SDKLIB_FILES} DESTINATION sdk/lib)

    # On Windows, we must copy Simbody libraries to the OpenSim build directory
    # so that the tests and examples can run without modifying the PATH
    # (that is, put Simbody's dll's in the same directory as OpenSim's
    # executables and libraries).
    if(WIN32)

        # This is where we're going to put these binaries.
        set(COPIED_LIB_FILES)
        foreach(LIBF ${SIMBODY_BIN_FILES})
            get_filename_component(LIBF_ROOT ${LIBF} NAME)
            set(COPIED_LIB_FILES ${COPIED_LIB_FILES}
                "${BUILD_BINARY_DIR}/${CMAKE_CFG_INTDIR}/${LIBF_ROOT}")
        endforeach()

        # This target depends on the destination copies of the Simbody files
        # (in the binary build directory).  Those are produced by the OUTPUT
        # custom command below, which depends on the source files from the
        # Simbody installation directory.
        add_custom_target(SimbodyFiles ALL DEPENDS ${COPIED_LIB_FILES})
        set_target_properties(SimbodyFiles
            PROPERTIES PROJECT_LABEL "Library - Simbody Files")

        foreach(LIBF ${SIMBODY_BIN_FILES})
            # message("LIB_FILE" ${LIBF})
            get_filename_component(LIBF_ROOT ${LIBF} NAME)
            get_filename_component(LIBF_SUFFIX ${LIBF} EXT)
            set(COPIED_LIBF "${BUILD_BINARY_DIR}/${CMAKE_CFG_INTDIR}/${LIBF_ROOT}")
            file(TO_NATIVE_PATH "${LIBF}" LIBF_SRC)
            file(TO_NATIVE_PATH "${COPIED_LIBF}" LIBF_DEST)

            # Copy Simbody files if they are out of date. This is invoked
            # because the SimbodyFiles target depends on these output files.
            add_custom_command(OUTPUT "${COPIED_LIBF}"
                COMMAND ${NATIVE_COPY_CMD} "${LIBF_SRC}" "${LIBF_DEST}"
                DEPENDS "${LIBF}"
                COMMENT "Copy ${LIBF_SRC} -> ${LIBF_DEST}"
                VERBATIM)
        endforeach()
    endif()
endif()


# Where do build products go in the build tree?
# ---------------------------------------------
if(NOT LIBRARY_OUTPUT_PATH)
    set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR} CACHE INTERNAL
        "Single output directory for building all libraries.")
endif()
if(NOT EXECUTABLE_OUTPUT_PATH)
  set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR} CACHE INTERNAL
      "Single output directory for building all executables.")
endif()


# Other than Windows we can debug without debuggable SimTK libraries
if(WIN32)
    set(CMAKE_DEBUG_POSTFIX "_d" CACHE INTERNAL "" FORCE)
else(WIN32)
    set(CMAKE_DEBUG_POSTFIX "_d" CACHE STRING "Suffix for debug libraries")
endif(WIN32)


## The following are required to uses Dart and the Cdash dashboard per Jesse
enable_testing()
include(CTest)

# Sets the number of CPUs testing would use
set(cmd ${CMAKE_CTEST_COMMAND} -j${PROCESSOR_COUNT})
if(MSVC)
    set(cmd ${cmd} -C ${CMAKE_CFG_INTDIR})
else(MSVC)
    set(cmd ${cmd} -C ${CMAKE_BUILD_TYPE})
endif(MSVC)
add_custom_target(RUN_TESTS_PARALLEL
    COMMAND ${cmd}
)


# Create buildinfo.txt file and place under sdk to include product version, platform and compiler for troubleshooting purposes
set(VERSION_FILE_PATH ${CMAKE_BINARY_DIR}/buildinfo.txt)
# message("version file="${VERSION_FILE_PATH})
file(WRITE ${VERSION_FILE_PATH} "Product Version=${OPENSIM_MAJOR_VERSION}.${OPENSIM_MINOR_VERSION}")
file(APPEND  ${VERSION_FILE_PATH} "\n")
file(APPEND  ${VERSION_FILE_PATH} "Compiler=${CMAKE_GENERATOR}-${CMAKE_CXX_COMPILER_ID}")
file(APPEND  ${VERSION_FILE_PATH} "\n")
file(APPEND  ${VERSION_FILE_PATH} "Platform=${Platform}-${PLATFORM_ABI}")
file(APPEND  ${VERSION_FILE_PATH} "\n")
install(FILES ${VERSION_FILE_PATH} DESTINATION sdk)


# Preprocessor definitions.
# -------------------------
# These are used in OpenSim/version.h
set(OPENSIM_SYSTEM_INFO ${CMAKE_SYSTEM})
set(OPENSIM_OS_NAME ${CMAKE_SYSTEM_NAME})

if( WIN32 )
    set(OPENSIM_COMPILER_INFO ${MSVC_VERSION})
else()
    set(OPENSIM_COMPILER_INFO ${CMAKE_CXX_COMPILER} )
endif()

add_definitions(-DOSIM_SYS_INFO=${OPENSIM_SYSTEM_INFO}
    -DOSIM_COMPILER_INFO=${OPENSIM_COMPILER_INFO}
    -DOSIM_OS_NAME=${OPENSIM_OS_NAME}
    -DOSIM_VERSION=${OPENSIM_VERSION})



#-----------------------------------------------------------------------------
set(BUILD_API_ONLY OFF CACHE BOOL "Build/install only headers, libraries,
    wrapping, tests; not applications (ik, rra, etc.).")
mark_as_advanced( BUILD_API_ONLY )

add_subdirectory(Vendors)
add_subdirectory(OpenSim)
if(NOT BUILD_API_ONLY)
    add_subdirectory(Applications)
endif()

add_subdirectory(cmake)
