#!/bin/bash
# 停止 Koch VAMP Server

echo "🛑 停止 VAMP Server..."

# 使用 pkill -9 直接且強制地終止 `koch_vamp_server` 服務
# 這與 start_vamp_server.sh 中的重啟邏輯一致
if pgrep -x "koch_vamp_server" > /dev/null; then
    echo "   找到運行中的伺服器，正在終止..."
    pkill -9 "koch_vamp_server"
    sleep 0.5
    echo "✅ Server 已關閉"
else
    echo "✅ 沒有運行中的 Server"
fi

# 清理舊的 socket 文件
echo "🧹 清理 socket 文件..."
rm -f /tmp/koch_vamp_server.sock
echo "✅ 清理完成"

exit 0
