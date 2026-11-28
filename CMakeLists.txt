cmake_minimum_required(VERSION 3.21)

project(formal_eskf VERSION 0.1.0 LANGUAGES CXX)

add_library(formal_eskf_scalar_math INTERFACE)
add_library(formal_eskf::scalar_math ALIAS formal_eskf_scalar_math)

target_include_directories(
    formal_eskf_scalar_math
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_compile_features(formal_eskf_scalar_math INTERFACE cxx_std_20)

add_library(formal_eskf_linalg INTERFACE)
add_library(formal_eskf::linalg ALIAS formal_eskf_linalg)

target_include_directories(
    formal_eskf_linalg
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_compile_features(formal_eskf_linalg INTERFACE cxx_std_20)
target_link_libraries(formal_eskf_linalg INTERFACE formal_eskf::scalar_math)

add_library(formal_eskf_so3 INTERFACE)
add_library(formal_eskf::so3 ALIAS formal_eskf_so3)

target_include_directories(
    formal_eskf_so3
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_compile_features(formal_eskf_so3 INTERFACE cxx_std_20)
target_link_libraries(formal_eskf_so3 INTERFACE formal_eskf::linalg)

add_library(formal_eskf_geodesy INTERFACE)
add_library(formal_eskf::geodesy ALIAS formal_eskf_geodesy)
target_link_libraries(formal_eskf_geodesy INTERFACE formal_eskf::linalg)

add_library(formal_eskf_core INTERFACE)
add_library(formal_eskf::core ALIAS formal_eskf_core)

target_include_directories(
    formal_eskf_core
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_compile_features(formal_eskf_core INTERFACE cxx_std_20)
target_link_libraries(formal_eskf_core INTERFACE formal_eskf::so3)

option(ESKF_CODEGEN "Use generated INS matrix arithmetic" ON)
option(ESKF_QUAT_APPROX "Use normalized Euler and first-order attitude error prediction" OFF)
option(ESKF_RESET_APPROX "Use first-order attitude-error covariance reset instead of the SO(3) right Jacobian" OFF)
option(ESKF_MAG_TILT "Restrict magnetic attitude correction to rotation about navigation Down" ON)
target_compile_definitions(
    formal_eskf_core
    INTERFACE
        ESKF_CODEGEN=$<BOOL:${ESKF_CODEGEN}>
        ESKF_QUAT_APPROX=$<BOOL:${ESKF_QUAT_APPROX}>
        ESKF_RESET_APPROX=$<BOOL:${ESKF_RESET_APPROX}>
        ESKF_MAG_TILT=$<BOOL:${ESKF_MAG_TILT}>
)

option(ESKF_ZUPT "Enable stationary INS zero-velocity updates" ON)
add_library(formal_eskf_runtime INTERFACE)
add_library(formal_eskf::runtime ALIAS formal_eskf_runtime)
target_compile_definitions(formal_eskf_runtime INTERFACE ESKF_ZUPT=$<BOOL:${ESKF_ZUPT}>)
target_link_libraries(formal_eskf_runtime INTERFACE formal_eskf::core formal_eskf::geodesy)

option(FORMAL_ESKF_BUILD_EIGEN_BACKEND "Build the Eigen linear-algebra backend" ON)

if(FORMAL_ESKF_BUILD_EIGEN_BACKEND)
    find_package(Eigen3 3.3 CONFIG REQUIRED)

    add_library(formal_eskf_linalg_eigen INTERFACE)
    add_library(formal_eskf::linalg_eigen ALIAS formal_eskf_linalg_eigen)
    target_link_libraries(
        formal_eskf_linalg_eigen
        INTERFACE
            formal_eskf::linalg
            Eigen3::Eigen
    )
endif()

option(FORMAL_ESKF_BUILD_PX4_MATRIX_BACKEND "Build the adapter for host-provided PX4 matrix" OFF)
set(FORMAL_ESKF_PX4_MATRIX_TARGET "" CACHE STRING "Optional existing target providing PX4 matrix's build environment")
set(FORMAL_ESKF_PX4_INCLUDE_DIRS "" CACHE STRING "Optional PX4 matrix, platform and generated-header include directories")

if(FORMAL_ESKF_BUILD_PX4_MATRIX_BACKEND)
    add_library(formal_eskf_linalg_px4_matrix INTERFACE)
    add_library(formal_eskf::linalg_px4_matrix ALIAS formal_eskf_linalg_px4_matrix)
    target_link_libraries(formal_eskf_linalg_px4_matrix INTERFACE formal_eskf::linalg)

    # PX4 owns the dependency. No submodule, download, or PX4 root build here.
    # An embedding PX4 build may already supply its includes/definitions.
    if(FORMAL_ESKF_PX4_MATRIX_TARGET)
        if(NOT TARGET ${FORMAL_ESKF_PX4_MATRIX_TARGET})
            message(FATAL_ERROR "FORMAL_ESKF_PX4_MATRIX_TARGET must name an existing CMake target")
        endif()
        target_link_libraries(formal_eskf_linalg_px4_matrix INTERFACE ${FORMAL_ESKF_PX4_MATRIX_TARGET})
    endif()
    if(FORMAL_ESKF_PX4_INCLUDE_DIRS)
        target_include_directories(formal_eskf_linalg_px4_matrix SYSTEM INTERFACE ${FORMAL_ESKF_PX4_INCLUDE_DIRS})
    endif()
endif()

option(FORMAL_ESKF_BUILD_CMSIS_DSP_BACKEND "Build the adapter for host-provided CMSIS-DSP" OFF)
set(FORMAL_ESKF_CMSIS_DSP_TARGET "CMSISDSP::CMSISDSP" CACHE STRING "Existing or installed target providing CMSIS-DSP")

if(FORMAL_ESKF_BUILD_CMSIS_DSP_BACKEND)
    # The embedding build owns CMSIS-DSP and its CPU/FPU/ABI/SIMD configuration.
    # Accept an existing target or an installed package; never fetch sources.
    if(FORMAL_ESKF_CMSIS_DSP_TARGET STREQUAL "CMSISDSP::CMSISDSP" AND
       NOT TARGET CMSISDSP::CMSISDSP)
        find_package(CMSISDSP CONFIG REQUIRED)
    endif()
    if(NOT TARGET "${FORMAL_ESKF_CMSIS_DSP_TARGET}")
        message(FATAL_ERROR "FORMAL_ESKF_CMSIS_DSP_TARGET must name an existing CMake target")
    endif()
    add_library(formal_eskf_linalg_cmsis_dsp INTERFACE)
    add_library(formal_eskf::linalg_cmsis_dsp ALIAS formal_eskf_linalg_cmsis_dsp)
    target_link_libraries(
        formal_eskf_linalg_cmsis_dsp
        INTERFACE
            formal_eskf::linalg
            ${FORMAL_ESKF_CMSIS_DSP_TARGET}
    )
endif()

option(FORMAL_ESKF_BUILD_EXAMPLES "Build host-side examples" OFF)

if(FORMAL_ESKF_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

if(PROJECT_IS_TOP_LEVEL)
    include(CTest)

    if(BUILD_TESTING)
        add_subdirectory(tests)
    endif()
endif()

# A misspelled project selection must not silently build the default path.
get_cmake_property(FORMAL_ESKF_CACHE_VARIABLES CACHE_VARIABLES)
foreach(FORMAL_ESKF_VARIABLE IN LISTS FORMAL_ESKF_CACHE_VARIABLES)
    if(FORMAL_ESKF_VARIABLE MATCHES "^(ESKF_|FORMAL_ESKF_)")
        get_property(FORMAL_ESKF_HELP CACHE "${FORMAL_ESKF_VARIABLE}" PROPERTY HELPSTRING)
        if(FORMAL_ESKF_HELP STREQUAL "No help, variable specified on the command line.")
            message(FATAL_ERROR "Unknown or inactive project setting: ${FORMAL_ESKF_VARIABLE}")
        endif()
        get_property(FORMAL_ESKF_TYPE CACHE "${FORMAL_ESKF_VARIABLE}" PROPERTY TYPE)
        string(TOUPPER "${${FORMAL_ESKF_VARIABLE}}" FORMAL_ESKF_VALUE)
        if(FORMAL_ESKF_TYPE STREQUAL "BOOL" AND NOT FORMAL_ESKF_VALUE MATCHES "^(ON|OFF|YES|NO|TRUE|FALSE|Y|N|0|1)$")
            message(FATAL_ERROR "Invalid boolean project setting: ${FORMAL_ESKF_VARIABLE}=${${FORMAL_ESKF_VARIABLE}}")
        endif()
    endif()
endforeach()
