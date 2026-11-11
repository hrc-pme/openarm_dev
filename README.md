# OpenArm Quick Start

**ROS 2 Humble · Docker · RViz2 · CAN FD · Dual-Arm Control**

本文件說明如何使用 `openarm_dev` 啟動開發環境、編譯 ROS 2 套件、測試雙手臂模擬，以及連接 OpenArm 實體硬體。

## 指令功能總覽

| 指令 / 程式 | 執行後的功能 |
| --- | --- |
| `bash run.sh` | 啟動並進入 OpenArm Docker 開發容器 |
| `bash ../colcon_build.sh` | 編譯 OpenArm CAN 通訊庫與 ROS 2 套件 |
| `source /root/openarm/environment.sh 0` | 載入 ROS 2 工作區並設定 `ROS_DOMAIN_ID=0` |
| `use_fake_hardware:=true` | 啟動雙手臂虛擬硬體，不控制實機 |
| `use_fake_hardware:=false` | 透過 CAN FD 連接實體 OpenArm |

---

## 1. 啟動 Docker 開發環境

**功能：** 建立／啟動 `openarm-dev` 容器，並進入容器的 Bash 終端機。專案目錄會掛載至容器內的 `/root/openarm`。

**執行位置：Ubuntu 主機**

```bash
git clone https://github.com/hrc-pme/openarm_dev.git
cd openarm_dev
bash run.sh
```

> 已下載專案者，只需進入專案目錄並執行 `bash run.sh`。

## 2. 編譯 ROS 2 套件

**功能：** 先編譯底層 `openarm_can`，再編譯其餘 ROS 2 套件，供模擬與實機控制使用。

**執行位置：Docker 容器（首次使用或程式更新後）**

```bash
cd /root/openarm/ros2_ws
bash ../colcon_build.sh
```

**載入編譯後的 ROS 2 環境：**

```bash
source /root/openarm/environment.sh 0
```

> `0` 是 ROS Domain ID；如需與其他電腦通訊，請確認雙方設定一致。注意：編譯腳本會刪除工作區的 `build/`、`install/`、`log/` 後重新編譯。

## 3. 啟動 OpenArm 模擬模式

**功能：** 啟動雙手臂機器人模型、虛擬硬體介面與 ROS 2 Controllers，可在沒有實體 OpenArm 的情況下進行測試，並使用 RViz2 檢視機器人狀態。

**執行位置：Docker 容器**

```bash
source /root/openarm/environment.sh 0
ros2 launch openarm_bringup openarm.bimanual.launch.py \
  arm_type:=v10 use_fake_hardware:=true
```

**重點：** `use_fake_hardware:=true` 不會向真實馬達發送控制指令。

## 4. 啟動 OpenArm 實體手臂

**功能：** 設定 CAN FD 介面，啟動 ROS 2 硬體驅動與 Controllers，接收雙手臂關節狀態，並提供控制介面。

### 4.1 設定 CAN FD

**執行位置：Ubuntu 主機**（需已接上並辨識 CAN 裝置）

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can0 up

sudo ip link set can1 down
sudo ip link set can1 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can1 up
```

確認 CAN 介面已啟用：

```bash
ip -details link show can0
ip -details link show can1
```

> 預設對應：`can0` 為右手臂、`can1` 為左手臂。實際接線必須與設定一致。

### 4.2 啟動實機控制

**執行位置：Docker 容器**

```bash
source /root/openarm/environment.sh 0
ros2 launch openarm_bringup openarm.bimanual.launch.py \
  arm_type:=v10 use_fake_hardware:=false \
  right_can_interface:=can0 left_can_interface:=can1
```

> **安全提醒：** 只在硬體版本、CAN 接線與供電確認無誤，且手臂運動範圍安全時使用實機模式。此指令會連接真實硬體，並非模擬。

## 5. 確認系統是否正常

**功能：** 查看 ROS 2 節點、Controller 狀態與機器人實際回傳的關節資料。

**執行位置：Docker 容器（另開終端機並進入容器）**

```bash
source /root/openarm/environment.sh 0

# 查看目前啟動的 ROS 2 節點
ros2 node list

# 查看 Controllers 是否載入／啟用
ros2 control list_controllers

# 查看關節狀態是否正常發布
ros2 topic echo /joint_states --once
```

## 常用參數

| 參數 | 功能 |
| --- | --- |
| `arm_type:=v10` | 選擇 OpenArm v1.0 硬體／模型配置 |
| `use_fake_hardware:=true` | 虛擬硬體模式 |
| `use_fake_hardware:=false` | 真實硬體模式 |
| `right_can_interface:=can0` | 指定右手臂 CAN 介面 |
| `left_can_interface:=can1` | 指定左手臂 CAN 介面 |

**相關文件：** [OpenArm ROS 2 Control](https://docs.openarm.dev/api-reference/ros2/control/) · [專案程式碼](https://github.com/hrc-pme/openarm_dev)
