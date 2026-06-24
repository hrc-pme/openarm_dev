#!/bin/bash
# Koch VAMP 即時規劃系統 - 啟動腳本

echo "=========================================="
echo "  Koch VAMP Real-time Planning System"
echo "=========================================="
echo ""

# 檢查伺服器是否已編譯
SERVER_PATH="/workspace/vamp/build/vamp_koch_vamp_server"
if [ ! -f "$SERVER_PATH" ]; then
    echo "❌ 伺服器未編譯！"
    echo "   請先執行："
    echo "   cd /workspace/vamp/build"
    echo "   cmake .. -DVAMP_BUILD_CPP_DEMO=ON"
    echo "   cmake --build . --target vamp_koch_vamp_server -j4"
    exit 1
fi

# 檢查是否已有伺服器在運行
if pgrep -x "vamp_koch_vamp_server" > /dev/null; then
    echo "⚠️  伺服器已在運行中"
    echo "   PID: $(pgrep -x 'vamp_koch_vamp_server')"
    echo ""
    read -p "是否要重啟？(y/N) " -n 1 -r
    echo ""
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "🛑 停止舊伺服器..."
        pkill -9 vamp_koch_vamp_server
        sleep 1
    else
        echo "繼續使用現有伺服器"
        exit 0
    fi
fi

# 清理舊的 socket 文件
rm -f /tmp/koch_vamp_server.sock

# 啟動伺服器
echo "🚀 啟動 C++ VAMP 伺服器..."
echo "   日誌: /tmp/vamp_server.log"
echo ""

cd /workspace/vamp/build
nohup ./vamp_koch_vamp_server > /tmp/vamp_server.log 2>&1 &
SERVER_PID=$!

# 等待伺服器啟動
echo "⏳ 等待伺服器啟動..."
for i in {1..10}; do
    if [ -S /tmp/koch_vamp_server.sock ]; then
        echo "✅ 伺服器已啟動！PID: $SERVER_PID"
        echo ""
        echo "📊 即時日誌："
        echo "   tail -f /tmp/vamp_server.log"
        echo ""
        echo "🤖 啟動 Python 客戶端："
        echo "   cd /workspace/koch_ros"
        echo "   python3 koch_vamp_client.py"
        echo ""
        echo "🛑 停止伺服器："
        echo "   kill $SERVER_PID"
        echo "   或 pkill vamp_koch_vamp_server"
        echo ""
        exit 0
    fi
    sleep 0.5
done

echo "❌ 伺服器啟動失敗！"
echo "   查看日誌: cat /tmp/vamp_server.log"
kill $SERVER_PID 2>/dev/null
exit 1
