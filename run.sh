#!/bin/bash

# 自動取得這個 script 所在的目錄 (projects/)
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

echo "🚀 Preparing OpenArm Docker environment (Root Mode)..."

# 執行 docker compose up -d (如果 Image 不存在它會自動 build，如果 Container 沒啟動它會啟動)
docker compose -f "$SCRIPT_DIR/docker/docker-compose.openarm.yml" up -d

echo "💻 Entering container [openarm-dev]..."
docker exec -it openarm-dev bash
