#!/bin/bash

# 這個腳本專門解決 `colcon build` 首次多封包平行編譯時，
# openarm_teleop 和 openarm_bringup 等上層封包會報錯找不到 OpenArmCAN 的問題。

# （Ubuntu 22.04 內建的 setuptools 太舊，不支援 --editable，會導致 Python 套件安裝失敗）
echo "📦 升級 Python setuptools..."
pip install -q --upgrade setuptools

echo "🧹 清理 realsense 的舊編譯快取（避免 symlink / directory 衝突）..."
rm -rf build/realsense2_camera_msgs build/realsense2_camera build/realsense2_ros_mqtt_bridge

echo "🔧 步驟一：開始單獨編譯底層硬體通訊庫 (openarm_can)..."
colcon build --packages-select openarm_can

echo "✅ openarm_can 編譯完成，載入其產生的環境設定..."
# 讓剛才編譯好的 openarm_can 目錄加入系統依賴中
source install/setup.bash

echo "🚀 步驟二：繼續編譯工作區內其餘所有的套件..."
# 注意：不使用 --symlink-install，因為 realsense2_ros_mqtt_bridge 的舊式 setup.py
# 不支援 pip 的 --editable 模式，加上 --symlink-install 會導致編譯失敗
colcon build

echo "🎉 所有 OpenArm 相關套件編譯完成！"
echo "👉 建議：請執行 'source install/setup.bash' 來確保所有環境參數都已載入當前終端機。"
