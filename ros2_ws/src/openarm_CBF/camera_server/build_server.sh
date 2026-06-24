#!/bin/bash
set -e

echo "=================================================="
echo "  VAMP Server 編譯 (支援 ROS 2)"
echo "=================================================="

BASE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
VAMP_DIR="$BASE_DIR/vamp"

echo "🔨 [1/2] 編譯 VAMP 基礎庫..."
cd "$VAMP_DIR"
mkdir -p build
cd build
# 關閉 DEMO 避免 VAMP 自行編譯 koch_vamp_server 時缺少 ROS 2 依賴
cmake .. -DVAMP_BUILD_PYTHON_BINDINGS=OFF \
         -DVAMP_BUILD_CPP_DEMO=OFF \
         -DVAMP_INSTALL_CPP_LIBRARY=ON
make -j$(nproc)

echo "🔨 [2/2] 編譯 ROS 2 節點 (koch_vamp_server)..."
cd "$BASE_DIR"
mkdir -p build_ros
cd build_ros

if [ -z "$ROS_DISTRO" ]; then
    source /opt/ros/humble/setup.bash || true
fi

# 執行上層的 CMakeLists.txt，這個 CMakeLists 會正確引入 rclcpp 與 sensor_msgs
cmake ..
make -j$(nproc)

if [ -f "koch_vamp_server" ]; then
    echo "✅ 編譯成功！"
    echo ""
    echo "安裝到系統..."
    sudo cp koch_vamp_server /usr/local/bin/
    sudo chmod +x /usr/local/bin/koch_vamp_server
    
    # 安裝啟動腳本
    sudo cp ../start_vamp_server.sh /usr/local/bin/start_vamp_server
    sudo cp ../stop_vamp_server.sh /usr/local/bin/stop_vamp_server
    sudo chmod +x /usr/local/bin/start_vamp_server /usr/local/bin/stop_vamp_server
    
    echo "=================================================="
    echo "✅ 安裝完成！"
    echo "執行檔: /usr/local/bin/koch_vamp_server"
else
    echo "❌ 編譯失敗"
    exit 1
fi
