cmake_minimum_required(VERSION 3.28...4.2)
project(common_robotics_utilities VERSION 0.1.0 LANGUAGES CXX)

include(CMakePackageConfigHelpers)
include(CheckIPOSupported)
include(CTest)
include(GNUInstallDirs)

option(BUILD_SHARED_LIBS "Build shared libraries" ON)
option(COMMON_ROBOTICS_UTILITIES_BUILD_EXAMPLES "Build the examples" ON)
option(COMMON_ROBOTICS_UTILITIES_NATIVE_ARCH
       "Compile with -march=native (only safe if every consumer does too)" OFF)

find_package(Eigen3 REQUIRED)
find_package(OpenMP)
find_package(ZLIB REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Defaults when this project is built on its own, as colcon does. A parent
# project or the command line can override them.
if(PROJECT_IS_TOP_LEVEL)
  if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
  endif()
  if(NOT DEFINED CMAKE_COMPILE_WARNING_AS_ERROR)
    set(CMAKE_COMPILE_WARNING_AS_ERROR ON)
  endif()
  if(NOT DEFINED CMAKE_INTERPROCEDURAL_OPTIMIZATION)
    check_ipo_supported(RESULT ipo_supported)
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION ${ipo_supported})
  endif()
endif()

# Warnings for this project's own targets only; consumers don't inherit them.
add_library(cru_warnings INTERFACE)
set(cru_warning_flags
  -Wall -Wextra -Wconversion -Wshadow -Wold-style-cast -Wpessimizing-move
  -Wuninitialized -Wmissing-declarations -Werror=non-virtual-dtor)
target_compile_options(cru_warnings INTERFACE
  "$<$<COMPILE_LANG_AND_ID:CXX,GNU,Clang,AppleClang>:${cru_warning_flags}>")

add_library(${PROJECT_NAME})
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_sources(${PROJECT_NAME}
  PRIVATE
    src/${PROJECT_NAME}/base64_helpers.cpp
    src/${PROJECT_NAME}/conversions.cpp
    src/${PROJECT_NAME}/math.cpp
    src/${PROJECT_NAME}/ros_conversions.cpp
    src/${PROJECT_NAME}/serialization.cpp
    src/${PROJECT_NAME}/time_optimal_trajectory_parametrization.cpp
    src/${PROJECT_NAME}/zlib_helpers.cpp
  PUBLIC
    FILE_SET HEADERS
    BASE_DIRS include
    FILES
      include/common_robotics_utilities/base64_helpers.hpp
      include/common_robotics_utilities/color_builder.hpp
      include/common_robotics_utilities/conversions.hpp
      include/common_robotics_utilities/cru_namespace.hpp
      include/common_robotics_utilities/dynamic_spatial_hashed_voxel_grid.hpp
      include/common_robotics_utilities/gaussian_distributions.hpp
      include/common_robotics_utilities/math.hpp
      include/common_robotics_utilities/maybe.hpp
      include/common_robotics_utilities/openmp_helpers.hpp
      include/common_robotics_utilities/parallelism.hpp
      include/common_robotics_utilities/path_processing.hpp
      include/common_robotics_utilities/print.hpp
      include/common_robotics_utilities/random_rotation_generator.hpp
      include/common_robotics_utilities/ros_conversions.hpp
      include/common_robotics_utilities/ros_helpers.hpp
      include/common_robotics_utilities/serialization.hpp
      include/common_robotics_utilities/simple_astar_search.hpp
      include/common_robotics_utilities/simple_dtw.hpp
      include/common_robotics_utilities/simple_graph.hpp
      include/common_robotics_utilities/simple_graph_search.hpp
      include/common_robotics_utilities/simple_hausdorff_distance.hpp
      include/common_robotics_utilities/simple_hierarchical_clustering.hpp
      include/common_robotics_utilities/simple_kmeans_clustering.hpp
      include/common_robotics_utilities/simple_knearest_neighbors.hpp
      include/common_robotics_utilities/simple_prm_planner.hpp
      include/common_robotics_utilities/simple_prngs.hpp
      include/common_robotics_utilities/simple_robot_model_interface.hpp
      include/common_robotics_utilities/simple_rrt_planner.hpp
      include/common_robotics_utilities/simple_task_planner.hpp
      include/common_robotics_utilities/time_optimal_trajectory_parametrization.hpp
      include/common_robotics_utilities/utility.hpp
      include/common_robotics_utilities/voxel_grid.hpp
      include/common_robotics_utilities/voxel_grid_common.hpp
      include/common_robotics_utilities/zlib_helpers.hpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    Eigen3::Eigen
    rclcpp::rclcpp
    ${geometry_msgs_TARGETS}
    ${visualization_msgs_TARGETS}
  PRIVATE
    ZLIB::ZLIB
    $<BUILD_INTERFACE:cru_warnings>)
if(OpenMP_CXX_FOUND)
  target_link_libraries(${PROJECT_NAME} PUBLIC OpenMP::OpenMP_CXX)
endif()
if(COMMON_ROBOTICS_UTILITIES_NATIVE_ARCH)
  target_compile_options(${PROJECT_NAME} PUBLIC -march=native)
endif()

if(COMMON_ROBOTICS_UTILITIES_BUILD_EXAMPLES)
  foreach(example clustering_example dtw_example)
    add_executable(${example} example/${example}.cpp)
    target_link_libraries(${example} PRIVATE ${PROJECT_NAME} cru_warnings)
  endforeach()
endif()

if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  include(GoogleTest)
  foreach(test
      hausdorff_distance_test
      math_test
      maybe_test
      parallelism_test
      planning_test
      print_test
      ros_helpers_test
      task_planning_test
      utility_test
      voxel_grid_test)
    add_executable(${test} test/${test}.cpp)
    target_link_libraries(${test}
      PRIVATE ${PROJECT_NAME} cru_warnings GTest::gtest_main)
    gtest_discover_tests(${test})
  endforeach()
endif()

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  FILE_SET HEADERS)
install(EXPORT ${PROJECT_NAME}Targets
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/cmake)
configure_package_config_file(
  cmake/${PROJECT_NAME}Config.cmake.in
  ${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/cmake)
write_basic_package_version_file(
  ${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  COMPATIBILITY SameMinorVersion)
install(FILES
    ${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    ${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/cmake)
install(FILES package.xml DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME})
