# Disable in-source builds to prevent source tree corruption
if("${CMAKE_CURRENT_SOURCE_DIR}" STREQUAL "${CMAKE_CURRENT_BINARY_DIR}")
    message(
        FATAL_ERROR
        "
FATAL: In-source builds are not allowed.
       You should create a separate directory for build files.
"
    )
endif()

cmake_minimum_required(VERSION 3.22)
set(CMAKE_MODULE_PATH
    ${CMAKE_MODULE_PATH}
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules"
)

# Set default build type to release with debug info (i.e. release mode
# optimizations are performed, but debug info still exists).
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "" FORCE)
endif()

project(TrajoptLib LANGUAGES CXX)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# Use, i.e. don't skip the full RPATH for the build tree
set(CMAKE_SKIP_BUILD_RPATH FALSE)

# When building, don't use the install RPATH already (but later on when
# installing)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)

# Add the automatically determined parts of the RPATH which point to directories
# outside the build tree to the install RPATH
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# The RPATH to be used when installing, but only if it's not a system directory
list(
    FIND
    CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES
    "${CMAKE_INSTALL_PREFIX}/lib"
    isSystemDir
)
if("${isSystemDir}" STREQUAL "-1")
    list(APPEND CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
endif()

# Generate compile_commands.json by default
if(NOT CMAKE_EXPORT_COMPILE_COMMANDS)
    set(CMAKE_EXPORT_COMPILE_COMMANDS "YES" CACHE STRING "" FORCE)
endif()

# Control where the static and shared libraries are built so that on Windows,
# we don't need to tinker with the path to run the executable
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")

option(BUILD_SHARED_LIBS "Build using shared libraries" ON)

set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS FALSE)

option(BUILD_EXAMPLES "Build examples" OFF)

include(CompilerFlags)

file(GLOB_RECURSE TrajoptLib_src src/*.cpp)
list(FILTER TrajoptLib_src EXCLUDE REGEX rust_ffi.cpp)

add_library(TrajoptLib ${TrajoptLib_src})
compiler_flags(TrajoptLib)
target_include_directories(TrajoptLib PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS TRUE)

set_target_properties(TrajoptLib PROPERTIES DEBUG_POSTFIX "d")

set_property(TARGET TrajoptLib PROPERTY FOLDER "libraries")
target_compile_definitions(TrajoptLib PRIVATE TRAJOPT_EXPORTS)

include(CTest)
include(FetchContent)

if(BUILD_TESTING)
    # Catch2 dependency
    FetchContent_Declare(
        Catch2
        GIT_REPOSITORY https://github.com/catchorg/Catch2.git
        GIT_TAG v3.8.1
        CMAKE_ARGS
    )
    FetchContent_MakeAvailable(Catch2)
endif()

set(BUILD_TESTING_SAVE ${BUILD_TESTING})
set(BUILD_EXAMPLES_SAVE ${BUILD_EXAMPLES})

set(BUILD_TESTING OFF)
set(BUILD_EXAMPLES OFF)

FetchContent_Declare(
    Sleipnir
    GIT_REPOSITORY https://github.com/SleipnirGroup/Sleipnir
    # main on 2025-06-06
    GIT_TAG 4e07c8835dc589e496a66d3320e0990668fea268
)
FetchContent_MakeAvailable(Sleipnir)

set(BUILD_TESTING ${BUILD_TESTING_SAVE})
set(BUILD_EXAMPLES ${BUILD_EXAMPLES_SAVE})

target_link_libraries(TrajoptLib PUBLIC Sleipnir)

target_include_directories(
    TrajoptLib
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

install(
    TARGETS TrajoptLib
    EXPORT TrajoptLibTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)
export(TARGETS TrajoptLib FILE TrajoptLib.cmake NAMESPACE TrajoptLib::)
install(DIRECTORY include/ COMPONENT TrajoptLib DESTINATION "include")
install(
    EXPORT TrajoptLibTargets
    FILE TrajoptLib.cmake
    NAMESPACE TrajoptLib::
    DESTINATION lib/cmake/TrajoptLib
)

include(CMakePackageConfigHelpers)

# Generate the config file that includes the exports
configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/TrajoptLibConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/TrajoptLibConfig.cmake
    INSTALL_DESTINATION "lib/cmake/TrajoptLib"
    NO_SET_AND_CHECK_MACRO
    NO_CHECK_REQUIRED_COMPONENTS_MACRO
)

# Install the config file
install(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/TrajoptLibConfig.cmake
    COMPONENT TrajoptLib
    DESTINATION lib/cmake/TrajoptLib
)

if(BUILD_TESTING)
    enable_testing()
    list(APPEND CMAKE_MODULE_PATH ${Catch2_SOURCE_DIR}/extras)
    include(Catch)
endif()
get_cmake_property(IS_MULTI_CONFIG GENERATOR_IS_MULTI_CONFIG)
# Build TrajoptLib tests
if(BUILD_TESTING)
    file(GLOB_RECURSE trajoptlib_test_src test/src/*.cpp)
    add_executable(trajoptlib_test ${trajoptlib_test_src})
    compiler_flags(trajoptlib_test)
    target_include_directories(
        trajoptlib_test
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/src
            ${CMAKE_CURRENT_SOURCE_DIR}/test/include
    )
    target_link_libraries(
        trajoptlib_test
        PRIVATE TrajoptLib Catch2::Catch2WithMain
    )
    catch_discover_tests(
        trajoptlib_test
        DL_PATHS ${Sleipnir_BINARY_DIR}/$<${IS_MULTI_CONFIG}:$<CONFIG>>
    )
endif()

# Build examples and example tests
if(BUILD_EXAMPLES)
    include(SubdirList)
    subdir_list(EXAMPLES ${CMAKE_CURRENT_SOURCE_DIR}/examples)
    foreach(example ${EXAMPLES})
        # Build example
        file(GLOB_RECURSE sources examples/${example}/src/*.cpp)
        add_executable(${example} ${sources})
        compiler_flags(${example})
        target_include_directories(
            ${example}
            PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/examples/${example}/include
        )
        target_link_libraries(${example} PRIVATE TrajoptLib)

        # Build example test if files exist for it
        if(
            BUILD_TESTING
            AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/examples/${example}/test
        )
            file(GLOB_RECURSE test_sources examples/${example}/test/*.cpp)
            add_executable(${example}_test ${sources} ${test_sources})
            target_include_directories(
                ${example}_test
                PRIVATE
                    ${CMAKE_CURRENT_SOURCE_DIR}/examples/${example}/src
                    ${CMAKE_CURRENT_SOURCE_DIR}/examples/${example}/test
            )
            compiler_flags(${example}_test)
            target_compile_definitions(${example}_test PUBLIC RUNNING_TESTS)
            target_include_directories(
                ${example}_test
                PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/examples/${example}/include
            )
            target_link_libraries(
                ${example}_test
                PRIVATE TrajoptLib Catch2::Catch2WithMain
            )
            if(NOT CMAKE_TOOLCHAIN_FILE)
                catch_discover_tests(
                    ${example}_test
                    DL_PATHS
                        ${Sleipnir_BINARY_DIR}/$<${IS_MULTI_CONFIG}:$<CONFIG>>
                )
            endif()
        endif()
    endforeach()
endif()
