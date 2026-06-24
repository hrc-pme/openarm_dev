# VAMP Server - 點雲避障處理伺服器

## 📋 概述

Koch VAMP Server 是一個獨立運行的點雲處理與路徑規劃伺服器，使用 RealSense 相機捕獲環境點雲，並提供避障路徑規劃服務。

## 🔧 安裝

### 一鍵安裝
```bash
cd /home/hrc/Lerobot_system/ros2_ws/src/canera_server
./build_server.sh
```

這會自動：
- 檢查依賴（RealSense SDK、Eigen3）
- 編譯 VAMP 庫
- 編譯 koch_vamp_server
- 安裝到 `/usr/local/bin/`

### 手動安裝依賴
```bash
# RealSense SDK
sudo apt-get install librealsense2-dev librealsense2-utils

# Eigen3
sudo apt-get install libeigen3-dev
```

## 📖 使用方法

### 啟動 Server
```bash
start_vamp_server
```

### 停止 Server
```bash
stop_vamp_server
```

### 查看日誌
```bash
tail -f /tmp/koch_vamp_server.log
```

### 檢查狀態
```bash
# 檢查進程
ps aux | grep koch_vamp_server

# 檢查 Socket
ls -la /tmp/koch_vamp_server.sock
```

## 🎯 功能特性

### 點雲處理
- **RealSense 深度相機**: 640x480 @ 30fps
- **Z-Order 過濾**: Morton Code 空間排序
- **地板過濾**: 自動過濾地面點 (z < 1cm)
- **自身過濾**: 自動過濾機器人本體 (5cm padding)

### 避障參數
```cpp
FILTER_RADIUS = 0.02f;         // 點雲過濾半徑 (2cm)
PHYSICAL_MARGIN = 0.03f;       // 物理安全邊距 (3cm)
OBSTACLE_TOTAL_RADIUS = 0.05f; // 總障礙物半徑 (5cm)
SELF_FILTER_PADDING = 0.05f;   // 手臂過濾範圍 (5cm)
```

### 相機配置
```cpp
CAM_X_OFFSET = 0.46f;    // 相機 X 偏移 (46cm)
CAM_Y_OFFSET = 0.00f;    // 相機 Y 偏移
CAM_Z_OFFSET = 0.12f;    // 相機 Z 偏移 (12cm)
FLOOR_THRESHOLD = 0.01f; // 地板過濾高度 (1cm)
```

## 🔌 Socket 通訊協議

### 請求格式 (PlanRequest)
```cpp
struct PlanRequest {
    float start[6];      // 起始關節角度 (rad)
    float goal[6];       // 目標關節角度 (rad)
    uint32_t max_iter;   // 最大迭代次數
    float range;         // RRT 搜索範圍
};
```

### 回應格式 (PlanResponse)
```cpp
struct PlanResponse {
    uint8_t success;      // 成功標誌 (0/1)
    uint32_t path_size;   // 路徑節點數量
    uint32_t cloud_size;  // 點雲數量
    uint32_t sphere_count;// 碰撞球數量
    uint32_t time_ms;     // 規劃時間 (ms)
};
```

後續資料：
- `path`: `float[path_size * 6]` - 路徑關節角度序列
- `cloud`: `float[cloud_size * 3]` - 過濾後的點雲 (x,y,z)
- `spheres`: `float[sphere_count * 4]` - 機器人碰撞球 (x,y,z,r)

## 🐛 故障排除

### 問題 1: "Camera error: No device connected"
```bash
# 檢查相機連接
lsusb | grep Intel
realsense-viewer

# 確認 udev rules
sudo cp /home/hrc/Lerobot_system/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### 問題 2: "Socket not detected"
```bash
# 清理舊 socket
rm -f /tmp/koch_vamp_server.sock
rm -f /tmp/koch_vamp_server.pid

# 重新啟動
start_vamp_server
```

### 問題 3: Server 崩潰
```bash
# 查看錯誤訊息
tail -100 /tmp/koch_vamp_server.log

# 檢查相機權限
groups | grep video  # 應包含 video 群組
```

## 📊 性能數據

- **點雲處理**: ~20ms (640x480)
- **路徑規劃**: 100-500ms (依複雜度)
- **記憶體使用**: ~200MB
- **CPU 使用**: 單核 30-50%

## 📝 檔案位置

```
/usr/local/bin/koch_vamp_server   # Server 執行檔
/usr/local/bin/start_vamp_server  # 啟動腳本
/usr/local/bin/stop_vamp_server   # 停止腳本
/tmp/koch_vamp_server.sock        # Unix socket
/tmp/koch_vamp_server.pid         # PID 檔案
/tmp/koch_vamp_server.log         # 日誌檔案
```

## 🔄 更新 Server

```bash
cd /home/hrc/Lerobot_system/ros2_ws/src/canera_server
stop_vamp_server  # 停止運行中的 server
./build_server.sh  # 重新編譯並安裝
start_vamp_server  # 啟動新版本
```

## 版本

- **V149** (2026-03-09) - Standalone 版本，RealSense 點雲處理


## 📋 概述

將 Koch VAMP Server（點雲避障規劃）整合到 ROS2 系統，支援與 teleoperation 配合使用。

## 🔧 系統架構

```
┌─────────────────────────────────────────┐
│  Teleoperation Launch Script           │
│  (leader_follower-teleop.sh)            │
└───────────┬─────────────────────────────┘
            │
            ├──► 1. VAMP Server Node (可選)
            │    ├─ vamp_server_node (ROS2 wrapper)
            │    └─ koch_vamp_server (C++ server)
            │       └─ RealSense 點雲處理
            │
            ├──► 2. Hardware Driver
            │    └─ koch_leader_follower_control
            │
            └──► 3. Teleop Bridge
                 └─ koch_teleop_bridge
```

## 📦 依賴套件

### 必需安裝：

1. **librealsense2** (RealSense SDK)
   ```bash
   sudo apt-get install librealsense2-dev librealsense2-utils
   ```

2. **VAMP** (路徑規劃庫) - 已包含在 `vamp/` 目錄

3. **ROS2 依賴**
   - rclcpp
   - std_msgs
   - sensor_msgs
   - geometry_msgs

## 🚀 安裝步驟

### 一鍵安裝（推薦）
```bash
cd /home/hrc/Lerobot_system/ros2_ws/src/canera_server
./setup_vamp.sh
```

### 手動安裝
```bash
# 1. 編譯 VAMP
cd /home/hrc/Lerobot_system/ros2_ws/src/canera_server/vamp
mkdir -p build && cd build
cmake .. -DVAMP_BUILD_PYTHON_BINDINGS=OFF \
         -DVAMP_BUILD_CPP_DEMO=OFF \
         -DVAMP_INSTALL_CPP_LIBRARY=ON
make -j$(nproc)

# 2. 編譯 ROS2 套件
cd /home/hrc/Lerobot_system/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select canera_server --merge-install

# 3. Source 環境
source install/setup.bash
```

## 📖 使用方法

### 方法 1: 單獨啟動 VAMP Server
```bash
# 啟動 VAMP Server (含 RealSense 相機)
ros2 launch canera_server vamp_server.launch.py

# 檢查狀態
ros2 topic echo /vamp_server/status
```

### 方法 2: 整合 Teleoperation (推薦)
```bash
cd /home/hrc/Lerobot_system/entrypoint
./leader_follower-teleop.sh
```

啟動時會詢問：
```
Enable VAMP Collision Avoidance? (y/N):
```
- 輸入 `y`: 啟用避障（啟動 VAMP Server）
- 輸入 `n`: 純 teleoperation（不啟動 VAMP）

## 🔍 系統檢查

### 檢查 RealSense 是否連接
```bash
realsense-viewer
```

### 檢查 VAMP Server 是否運行
```bash
# 檢查進程
ps aux | grep koch_vamp_server

# 檢查 Socket
ls -la /tmp/koch_vamp_server.sock
```

### 檢查 ROS2 節點
```bash
ros2 node list
# 應看到: /vamp_server_node

ros2 topic list
# 應看到: /vamp_server/status
```

## ⚙️ 配置參數

編輯 `launch/vamp_server.launch.py` 修改參數：

```python
parameters=[
    {'socket_path': '/tmp/koch_vamp_server.sock'},        # Socket 路徑
    {'auto_start_server': True},                          # 自動啟動 C++ server
    {'status_check_interval_ms': 5000}                    # 狀態檢查間隔 (ms)
]
```

## 🐛 故障排除

### 問題 1: "Camera error: No device connected"
**解決方案:**
```bash
# 檢查 RealSense 連接
lsusb | grep Intel
realsense-viewer

# 確認 udev rules
sudo cp /home/hrc/Lerobot_system/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### 問題 2: "VAMP not built!"
**解決方案:**
```bash
cd /home/hrc/Lerobot_system/ros2_ws/src/canera_server
./setup_vamp.sh
```

### 問題 3: "Socket not detected"
**解決方案:**
```bash
# 清理舊 socket
rm -f /tmp/koch_vamp_server.sock

# 重新啟動
ros2 launch canera_server vamp_server.launch.py
```

## 📊 性能參數

從 `koch_vamp_server.cc` 配置：

```cpp
// 避障參數
FILTER_RADIUS = 0.02f;         // 點雲過濾半徑 (2cm)
PHYSICAL_MARGIN = 0.03f;       // 物理安全邊距 (3cm)
OBSTACLE_TOTAL_RADIUS = 0.05f; // 總障礙物半徑 (5cm)
SELF_FILTER_PADDING = 0.05f;   // 手臂自身過濾範圍 (5cm)

// 相機設置
CAM_X_OFFSET = 0.46f;          // 相機 X 偏移 (46cm)
CAM_Y_OFFSET = 0.00f;          // 相機 Y 偏移
CAM_Z_OFFSET = 0.12f;          // 相機 Z 偏移 (12cm)
FLOOR_THRESHOLD = 0.01f;       // 地板過濾高度 (1cm)
```

## 🎯 未來擴展

- [ ] 發布點雲到 ROS2 topic (`sensor_msgs/PointCloud2`)
- [ ] 發布碰撞球到 RViz (`visualization_msgs/MarkerArray`)
- [ ] 整合動態重配置 (`rqt_reconfigure`)
- [ ] 添加規劃路徑可視化
- [ ] 支援多相機融合

## 📝 版本歷史

- **V0.0.1** (2026-03-09)
  - ROS2 整合完成
  - 支援 teleoperation 可選啟用
  - 自動進程管理
