OpenArm Quick Start
ROS 2 Humble · Docker · RViz Simulation · CAN FD Hardware Control
1. Docker 啟動
在 Ubuntu 主機執行：
git clone https://github.com/hrc-pme/openarm_dev.git   # 已下載可略過
cd openarm_dev                                      # 或進入既有專案目錄
bash run.sh
- 自動建立／啟動 openarm-dev 容器並進入 Bash。
- 容器內專案路徑：`/root/openarm`。
2. ROS 2 編譯（首次使用）
在 Docker 容器內執行：
cd /root/openarm/ros2_ws
bash ../colcon_build.sh
source /root/openarm/environment.sh 0
- 編譯腳本會先編譯 openarm_can，再編譯其他套件。
- 注意： 此腳本會清除原本的 build/、`install/`、`log/`。
3. RViz 模擬
在 Docker 容器內執行：
source /root/openarm/environment.sh 0
ros2 launch openarm_bringup openarm.bimanual.launch.py arm_type:=v10 use_fake_hardware:=true
4. OpenArm 實機啟動
① Ubuntu 主機： 確認硬體、急停及 CAN 介面對應後，設定 can0、`can1`：
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can0 up
sudo ip link set can1 down
sudo ip link set can1 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can1 up
② Docker 容器： 啟動實機控制：
source /root/openarm/environment.sh 0
ros2 launch openarm_bringup openarm.bimanual.launch.py arm_type:=v10 use_fake_hardware:=false
實機啟動前，請確認 arm_type:=v10 符合硬體版本，並確保機器人運動範圍安全。

5. 狀態檢查
在 Docker 容器內執行：
ros2 node list
ros2 control list_controllers
ros2 topic echo --once /joint_states
- use_fake_hardware:=true：模擬模式
- use_fake_hardware:=false：實機模式
- environment.sh 0 中的 0 為 ROS_DOMAIN_ID，可依需求修改。
