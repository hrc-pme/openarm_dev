#!/bin/bash
# 停止 Koch VAMP Server

echo "🛑 停止 VAMP Server..."

# 找到所有 vamp_koch_vamp_server 進程
PIDS=$(pgrep -f vamp_koch_vamp_server)

if [ -z "$PIDS" ]; then
    echo "✅ 沒有運行中的 Server"
    rm -f /tmp/koch_vamp_server.sock
    exit 0
fi

echo "   找到進程: $PIDS"

# 直接使用 kill -9 強制終止所有進程
for pid in $PIDS; do
    echo "   強制終止 PID: $pid"
    kill -9 $pid 2>/dev/null
done

sleep 1

# 確認已關閉
REMAINING=$(pgrep -f vamp_koch_vamp_server)
if [ -n "$REMAINING" ]; then
    echo "❌ Server 仍在運行: $REMAINING"
    echo "   嘗試使用 killall..."
    killall -9 vamp_koch_vamp_server 2>/dev/null
    sleep 1
fi

# 最終確認
if pgrep -f vamp_koch_vamp_server > /dev/null; then
    echo "❌ 無法終止 Server！"
    exit 1
else
    echo "✅ Server 已關閉"
    # 清理 socket 文件
    rm -f /tmp/koch_vamp_server.sock
    echo "✅ 清理完成"
fi
