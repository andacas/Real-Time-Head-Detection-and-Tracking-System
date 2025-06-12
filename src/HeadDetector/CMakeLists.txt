cmake_minimum_required(VERSION 3.16)
project(head_detector LANGUAGES C CXX)

# ─── build flags ──────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic -march=native -O3 -ffast-math)

# ─── ROS 2 + libs ─────────────────────────────────────────────────────
find_package(ament_cmake          REQUIRED)
find_package(rclcpp               REQUIRED)
find_package(sensor_msgs          REQUIRED)
find_package(visualization_msgs   REQUIRED)
find_package(pcl_conversions      REQUIRED)
find_package(tf2                  REQUIRED)          # ▶ quaternion
find_package(tf2_ros              REQUIRED)          
find_package(tf2_sensor_msgs              REQUIRED)          
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(Eigen3               REQUIRED)
find_package(OpenMP               REQUIRED)

# ─── GPU off-load flags (same logic as before) ────────────────────────
set(GPU_SM 70)  # change for your card
set(HAVE_GPU 0)
if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  include(CheckCXXCompilerFlag)
  check_cxx_compiler_flag(
    "-foffload=nvptx-none=-march=sm_${GPU_SM}" HAS_NVPTX)
  if (HAS_NVPTX)
    add_compile_options(-fopenmp
                        -foffload=nvptx-none=-march=sm_${GPU_SM})
    set(HAVE_GPU 1)
  else()
    add_compile_options(-fopenmp)
  endif()
elseif (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-fopenmp
                      -fopenmp-targets=nvptx64-nvidia-cuda
                      -Xopenmp-target -march=sm_${GPU_SM})
  set(HAVE_GPU 1)
else()
  add_compile_options(-fopenmp)
endif()
add_definitions(-DHAVE_GPU=${HAVE_GPU})

# ─── include dirs for our headers ─────────────────────────────────────
include_directories(
  src                             # detection/…
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# ─── executable node ─────────────────────────────────────────────────
add_executable(head_detector_node
  src/head_detector_node.cpp)

# ament will add include-dirs & link flags for these packages:
ament_target_dependencies(head_detector_node
  rclcpp
  sensor_msgs
  visualization_msgs
  pcl_conversions
  tf2                   #   ⬅ already there
  tf2_ros               #   ⬅ add
  tf2_sensor_msgs)      #   ⬅ add

# extra libs not known by ament (keep as-is)
target_link_libraries(head_detector_node
  OpenMP::OpenMP_CXX
  ${PCL_LIBRARIES}
  Eigen3::Eigen)


  # ───────────────────── new demo / utility executables ─────────────────────
add_executable(head_detector_cli      src/main.cpp)
add_executable(head_detector_realtime src/main_ply_realtime.cpp)

# Both demos only depend on PCL and the header-only Detector
find_package(PCL REQUIRED COMPONENTS io visualization common)

target_link_libraries(head_detector_cli      ${PCL_LIBRARIES})
target_link_libraries(head_detector_realtime ${PCL_LIBRARIES})

ament_target_dependencies(head_detector_cli
  PCL
)

ament_target_dependencies(head_detector_realtime
  PCL
)

# Install so `ros2 run head_detector <exec>` works
install(TARGETS
  head_detector_cli
  head_detector_realtime
  DESTINATION lib/${PROJECT_NAME}
)
# ───────────────────────────────────────────────────────────────────────────


# ─── install ─────────────────────────────────────────────────────────
install(TARGETS head_detector_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY src/detection
  DESTINATION include
  FILES_MATCHING PATTERN "*.h*")

ament_package()
