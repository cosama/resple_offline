cmake_minimum_required(VERSION 3.22)
project(resple_offline LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io kdtree filters)
find_package(OpenMP)
set(PYBIND11_FINDPYTHON ON)
find_package(pybind11 CONFIG REQUIRED)

# Pinned upstream + auditable patch series
# Apply checked-in patches to a build-tree copy, leaving the submodule pristine.
get_filename_component(RESPLE_BRIDGE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
set(UPSTREAM_RESPLE "${RESPLE_BRIDGE_ROOT}/upstream/RESPLE/resple")
if(NOT EXISTS "${UPSTREAM_RESPLE}/src/RESPLE.cpp")
  message(FATAL_ERROR
    "Pinned upstream RESPLE source is missing at ${UPSTREAM_RESPLE}. "
    "Run: git submodule update --init upstream/RESPLE")
endif()

set(RESPLE_STAGE "${CMAKE_CURRENT_BINARY_DIR}/upstream_staged")
file(REMOVE_RECURSE "${RESPLE_STAGE}")
file(MAKE_DIRECTORY "${RESPLE_STAGE}")
file(COPY "${UPSTREAM_RESPLE}/include" "${UPSTREAM_RESPLE}/src"
     DESTINATION "${RESPLE_STAGE}")

find_program(PATCH_EXECUTABLE patch REQUIRED)
file(GLOB RESPLE_INTEGRATION_PATCHES
     "${CMAKE_CURRENT_SOURCE_DIR}/patches/integration/*.patch")
list(SORT RESPLE_INTEGRATION_PATCHES)
list(LENGTH RESPLE_INTEGRATION_PATCHES RESPLE_PATCH_COUNT)
if(RESPLE_PATCH_COUNT EQUAL 0)
  message(FATAL_ERROR "No patches found under patches/integration; the delta must be explicit")
endif()

foreach(patch_file IN LISTS RESPLE_INTEGRATION_PATCHES)
  execute_process(
    # Reject upstream drift instead of relocating mismatched hunks with fuzz.
    COMMAND "${PATCH_EXECUTABLE}" -F0 --silent --no-backup-if-mismatch -p0 -i "${patch_file}"
    WORKING_DIRECTORY "${RESPLE_STAGE}"
    RESULT_VARIABLE patch_result
    ERROR_VARIABLE patch_error)
  if(NOT patch_result EQUAL 0)
    message(FATAL_ERROR
      "Patch did not apply to pinned upstream: ${patch_file}\n${patch_error}\n"
      "The upstream revision has drifted from the pin recorded in UPSTREAM.md.")
  endif()
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${patch_file}")
endforeach()
message(STATUS "RESPLE: applied ${RESPLE_PATCH_COUNT} patches to staged upstream")

# Native module
# bindings.cpp includes RESPLE.cpp because upstream defines process-global
# state in its headers; keeping it in one translation unit avoids ODR violations.
pybind11_add_module(_core NO_EXTRAS
  cpp/bindings.cpp
  "${RESPLE_STAGE}/include/ikd-Tree/ikd_Tree.cpp"
)

set_source_files_properties(cpp/bindings.cpp PROPERTIES
  COMPILE_OPTIONS "-Wno-unused-variable;-Wno-unused-but-set-variable;-Wno-sign-compare;-Wno-reorder")
set_source_files_properties("${RESPLE_STAGE}/include/ikd-Tree/ikd_Tree.cpp" PROPERTIES
  COMPILE_OPTIONS "-Wno-unused-variable;-Wno-unused-but-set-variable;-Wno-sign-compare;-Wno-maybe-uninitialized")

target_include_directories(_core PRIVATE
  cpp/compat                      # shadows rclcpp/sensor_msgs/.../pcl_conversions
  cpp/include                     # bridge ABI (resple_bridge/offline_bridge.hpp)
  "${RESPLE_STAGE}/include"       # Estimator.h, SplineState.h, Association.h, utils/, ikd-Tree/
  "${RESPLE_STAGE}/src"
  ${PCL_INCLUDE_DIRS}
)

target_compile_definitions(_core PRIVATE
  EIGEN_MALLOC_ALREADY_ALIGNED=1
  EIGEN_INITIALIZE_MATRICES_BY_NAN=1
)
target_compile_options(_core PRIVATE -O3 -Wall)

target_link_libraries(_core PRIVATE Eigen3::Eigen ${PCL_LIBRARIES})
if(OpenMP_CXX_FOUND)
  target_link_libraries(_core PRIVATE OpenMP::OpenMP_CXX)
endif()

install(TARGETS _core LIBRARY DESTINATION resple)
