#!/bin/bash
# VAMP Server 編譯腳本 (Standalone)

set -e

echo "=================================================="
echo "  VAMP Server 編譯"
echo "=================================================="

# 1. 檢查 RealSense SDK
echo "📦 [1/3] 檢查 RealSense SDK..."
if pkg-config --exists realsense2; then
    VERSION=$(pkg-config --modversion realsense2)
    echo "✅ librealsense2 已安裝 (版本: $VERSION)"
else
    echo "❌ librealsense2 未安裝"
    echo "執行以下命令安裝："
    echo "  sudo apt-get install librealsense2-dev librealsense2-utils"
    exit 1
fi

# 2. 編譯 VAMP
echo ""
echo "🔧 [2/3] 檢查 VAMP 函式庫..."
VAMP_DIR="/home/hrc/Lerobot_system/ros2_ws/src/canera_server/vamp"
if [ ! -d "$VAMP_DIR/build" ]; then
    echo "⚙️  VAMP 尚未編譯，開始編譯..."
    cd "$VAMP_DIR"
    mkdir -p build
    cd build
    cmake .. -DVAMP_BUILD_PYTHON_BINDINGS=OFF \
             -DVAMP_BUILD_CPP_DEMO=OFF \
             -DVAMP_INSTALL_CPP_LIBRARY=ON
    make -j$(nproc)
    echo "✅ VAMP 編譯完成"
else
    echo "✅ VAMP 已編譯"
fi

# 3. 編譯 koch_vamp_server
echo ""
echo "🔨 [3/3] 編譯 koch_vamp_server..."
cd /home/hrc/Lerobot_system/ros2_ws/src/canera_server
mkdir -p build
cd build
cmake ..
make -j$(nproc)

# 4. 安裝到系統
echo ""
echo "📦 安裝執行檔..."
sudo make install

echo ""
echo "=================================================="
echo "✅ VAMP Server 編譯完成！"
echo "=================================================="
echo ""
echo "執行檔位置: /usr/local/bin/koch_vamp_server"
echo ""
echo "使用方法："
echo "1. 直接執行:"
echo "   koch_vamp_server"
echo ""
echo "2. 使用啟動腳本:"
echo "   /usr/local/bin/start_vamp_server.sh"
echo ""
echo "3. 停止 server:"
echo "   /usr/local/bin/stop_vamp_server.sh"
echo ""
