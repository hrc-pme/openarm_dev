# OpenArm Local Development Environment Guide

 OpenArm 專案開發環境。本專案使用 Docker 容器封裝完整的 ROS 2 Humble 開發環境。

## 📦 環境包含

- **ROS 2 Humble Desktop Full**（含 Rviz2、MoveIt2 等工具）
- **Intel RealSense SDK v2.56.1**
- **PyTorch**（預設安裝 CUDA 12.8 版本；不同版本，請修改 `docker/dockerfile.openarm` 中的 `--index-url`）

---

## 🚀 1. 啟動並進入開發環境

開發前，請在 `openarm/` 根目錄執行啟動腳本：

```bash
source run.sh
```

> 這個腳本會自動判斷 Docker Image 是否已建立，若未建立則會先執行 Build 再啟動。本機的 `openarm/` 資料夾會掛載至容器內的 `/root/openarm`。

---

## 🔧 2. 首次編譯 ROS 2 工作區

進入容器後（提示字元變為 `root@docker-desktop` 或類似），請使用我們提供的專用編譯腳本：

```bash
cd /root/openarm/ros2_ws
bash ../colcon_build.sh
```

**⚠️ 為什麼不能直接 `colcon build`？**
`openarm_teleop` 依賴 `openarm_can`，但 colcon 預設的平行編譯會導致 `openarm_can` 還沒編譯完前就已經找不到 `OpenArmCAN` 的問題。這個腳本會：
1. 先單獨編譯 `openarm_can`
2. 自動 `source install/setup.bash` 讓依賴生效
3. 再一次編譯其餘所有套件

---

## 🌐 3. 設定 ROS 2 Domain ID

執行任何 `ros2 run` 或 `ros2 launch` 前，請設定 ROS 2 DOMAIN ID：

```bash
source /root/openarm/environment.sh <ROS_DOMAIN_ID>
```

- 若未輸入 ID，預設使用 `ROS_DOMAIN_ID=0`
- 此腳本同時也會 `source /opt/ros/humble/setup.bash`

---

## 🤖 4. 執行範例

所有前置作業完成後，即可啟動所需節點。例如測試 MoveIt2 雙臂控制：

```bash
cd /root/openarm/ros2_ws && source install/setup.bash
ros2 launch openarm_bringup openarm.launch.py arm_type:=v10 use_fake_hardware:=true
```

如果要控制真機則在docker內先設定can0 & can1再launch
```bash
sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on && sudo ip link set can0 up
sudo ip link set can1 type can bitrate 1000000 dbitrate 5000000 fd on && sudo ip link set can1 up
```

*(圖形化介面如 rviz2 會透過 X11 顯示在本機螢幕)*

---

## 📁 專案結構說明

| 路徑 | 說明 |
|---|---|
| `run.sh` | 啟動並進入 Docker 容器的主腳本 |
| `colcon_build.sh` | 首次編譯時使用的有序編譯腳本 |
| `environment.sh` | 設定 ROS Domain ID 及環境的腳本 |
| `docker/dockerfile.openarm` | Docker Image 定義（含 ROS、librealsense、PyTorch） |
| `docker/docker-compose.openarm.yml` | Docker Compose 設定（Volume 掛載、GPU 等） |
| `ros2_ws/src/` | ROS 2 工作區原始碼（openarm_ros2、realsense-ros 等） |

---

### 目錄結構

```text
openarm/
├── docker
│   ├── docker-compose.openarm.yml
│   └── dockerfile.openarm
├── ros2_ws
│   └── src
│       ├── openarm_can
│       ├── openarm_description
│       ├── openarm_isaac_lab
│       ├── openarm_ros2
│       ├── openarm_teleop
│       └── realsense-ros
├── run.sh
├── colcon_build.sh
├── README.md
└── environment.sh
```