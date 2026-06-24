cmake_minimum_required(VERSION 3.16)
project(canera_server)

# C++17 標準
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

# 尋找 ROS 2 套件
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# 尋找 RealSense
find_package(realsense2 REQUIRED)

# 尋找 Eigen3
find_package(Eigen3 REQUIRED)

# VAMP 路徑
set(VAMP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/vamp)
set(VAMP_INCLUDE ${VAMP_DIR}/src/impl)
set(VAMP_BUILD ${VAMP_DIR}/build)

# 檢查 VAMP 是否已編譯
if(NOT EXISTS ${VAMP_BUILD})
  message(FATAL_ERROR "VAMP not built! Run: cd ${VAMP_DIR} && mkdir -p build && cd build && cmake .. && make")
endif()

# ====== Compiler flags =======
add_compile_options(-mavx -mavx2 -mfma -O3)

# ===== VAMP Server 可執行檔 (standalone) =====
add_executable(koch_vamp_server
  vamp/scripts/cpp/koch_vamp_server.cc
)

target_include_directories(koch_vamp_server PRIVATE
  ${VAMP_INCLUDE}
  ${VAMP_BUILD}/_deps/pdqsort-src
  ${VAMP_BUILD}/_deps/nigh-src/src
  ${VAMP_BUILD}/_deps/simdxorshift-src/include
  ${realsense2_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(koch_vamp_server
  ${realsense2_LIBRARY}
  ${VAMP_BUILD}/libsimdxorshift.a
  pthread
)

ament_target_dependencies(koch_vamp_server rclcpp sensor_msgs)


# ===== 安裝 =====
install(TARGETS
  koch_vamp_server
  DESTINATION bin
)

install(PROGRAMS
  start_vamp_server.sh
  stop_vamp_server.sh
  DESTINATION bin
)
