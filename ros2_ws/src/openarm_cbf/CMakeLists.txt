cmake_minimum_required(VERSION 3.16)
project(openarm_cbf)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(realsense2 REQUIRED)
find_package(Eigen3 REQUIRED)

# VAMP 路徑設定
set(VAMP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/camera_server/vamp)
set(VAMP_INCLUDE ${VAMP_DIR}/src/impl)
set(VAMP_BUILD ${CMAKE_BINARY_DIR}/vamp_build)

# --------------------------------------------------------------------
# 將 VAMP 作為子專案編譯 (add_subdirectory)
# --------------------------------------------------------------------
# 這種方法比 ExternalProject 更穩健，因為它將 VAMP 的 target 直接整合到主建置系統中。

# 設定 VAMP 的編譯選項，避免與 ROS package 衝突
# 我們只想要 VAMP 的 C++ 核心函式庫，不要它自己的 Python 綁定或安裝規則
set(VAMP_BUILD_PYTHON_BINDINGS OFF CACHE BOOL "Disable python bindings in subproject" FORCE)
set(VAMP_INSTALL_CPP_LIBRARY OFF CACHE BOOL "Disable installation in subproject" FORCE)
set(VAMP_BUILD_CPP_DEMO OFF CACHE BOOL "Disable demos in subproject" FORCE)
set(VAMP_BUILD_OMPL_DEMO OFF CACHE BOOL "Disable OMPL demos in subproject" FORCE)

# 將 VAMP 作為子目錄加入，這會讓它的 target (例如 vamp::vamp) 在我們的主專案中可用
add_subdirectory(${VAMP_DIR} ${VAMP_BUILD})

# --------------------------------------------------------------------
# VAMP Server 可執行檔
# --------------------------------------------------------------------
add_compile_options(-mavx -mavx2 -mfma -O3)

add_executable(koch_vamp_server
  camera_server/vamp/scripts/cpp/koch_vamp_server.cc
)

# vamp::vamp 是一個 INTERFACE library，它會自動帶入它自己的 include 路徑。
# 我們只需要為 koch_vamp_server 明確加入它直接使用的函式庫的 include 路徑。
target_include_directories(koch_vamp_server PRIVATE
  ${realsense2_INCLUDE_DIR}
)

# 連結 VAMP 函式庫 (vamp::vamp) 和其他需要的函式庫。
# vamp::vamp 會自動處理其依賴項 (如 simdxorshift, Eigen3 等)。
target_link_libraries(koch_vamp_server
  vamp::vamp
  ${realsense2_LIBRARY}
  pthread
)

# 使用 ament_target_dependencies 來處理 ROS 2 相關的依賴，這是標準做法
ament_target_dependencies(koch_vamp_server
  rclcpp
  sensor_msgs
)

# ===== 安裝設定 =====
install(
    PROGRAMS
    open_CBF.py
    test_urdf.py
    DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  koch_vamp_server
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  camera_server/start_vamp_server.sh
  camera_server/stop_vamp_server.sh
  DESTINATION bin
)

ament_package()