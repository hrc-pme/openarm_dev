OpenArm — Docker Setup & Hardware Bringup Guide
本專案提供 OpenArm 雙臂機器人的 ROS 2 開發環境，說明如何透過 Docker 啟動 ROS 2 Humble、編譯工作區，並以**模擬模式（Fake Hardware）或實機模式（Real Hardware）**啟動 OpenArm。
重要： Docker 提供執行環境；真正與機器人通訊需要主機辨識到 can0、`can1`，並正確設定 CAN FD。第一次操作實機前，請確認機械臂周圍安全、緊急停止裝置可用，且機器人型號與啟動參數一致。

1. 系統需求
- 已安裝 Docker Engine 與 Docker Compose Plugin 的 Ubuntu 主機。
- OpenArm ROS 2 工作區（包含本專案的 ros2_ws/src/）。
- 實機模式另需： OpenArm 雙臂、對應的 CAN FD 介面（can0、`can1`）及正確的電源／線路連接。
- 若要使用 RViz2，需有可用的圖形顯示環境（X11）。
本專案的 Docker Compose 設定使用 network_mode: host、`privileged: true`，並掛載主機的 /dev 與 X11 socket，讓容器可以使用 ROS 2 網路通訊、硬體裝置及圖形介面。
2. 啟動 Docker 開發環境
以下指令在 Ubuntu 主機（Host） 執行：
git clone https://github.com/hrc-pme/openarm_dev.git
cd openarm_dev
bash run.sh
run.sh 會透過 Docker Compose 建立／啟動 openarm-dev 容器，並進入容器的 Bash。專案在容器內掛載至：
/root/openarm
若容器已經啟動，之後也可以從主機重新進入：
docker exec -it openarm-dev bash
以下未特別標註的 ROS 2 指令，皆在 Docker 容器內 執行。

3. 首次編譯 ROS 2 工作區
在 Docker 容器內 執行：
cd /root/openarm/ros2_ws
bash ../colcon_build.sh
source install/setup.bash
colcon_build.sh 會先編譯 openarm_can，再編譯其餘套件，解決初次建置時的依賴順序問題。

注意： 目前的 colcon_build.sh 會刪除工作區中的 build/、`install/`、`log/` 後重新編譯，請勿將它當作每次啟動都必須執行的指令。

編譯完成後，設定 ROS 2 環境與 Domain ID：
source /root/openarm/environment.sh 0
0 為範例 Domain ID；若需要和其他 ROS 2 主機通訊，請讓相關主機使用相同的 ROS_DOMAIN_ID。不傳入參數時，此腳本預設使用 0。
4. 模擬模式：Fake Hardware
建議先用模擬模式確認 ROS 2 套件能正常啟動，不需連接實體機器人，也不需設定 CAN 介面。

在 Docker 容器內 執行：
source /root/openarm/environment.sh 0
ros2 launch openarm_bringup openarm.bimanual.launch.py arm_type:=v10 use_fake_hardware:=true
此模式使用 Fake Hardware 介面驗證 Bringup 流程。若有啟動 RViz2，應能查看機器人模型與狀態；是否自動開啟圖形介面，以實際 launch 檔案設定為準。
arm_type:=v10 沿用本專案原有範例。若實際使用的 OpenArm 硬體版本不同，請先確認對應的支援參數。

5. 實機模式：Real Hardware
5.1 確認 CAN FD 介面
先在 Ubuntu 主機（Host） 檢查：
ip -br link show can0
ip -br link show can1
如果沒有 can0 或 can1，先確認 CAN 轉接器、驅動及裝置名稱；不要直接進入實機啟動步驟。
5.2 設定 CAN FD
在 Ubuntu 主機（Host） 執行以下指令（沿用專案原本的 CAN FD 參數）：
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can0 up

sudo ip link set can1 down
sudo ip link set can1 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can1 up
確認設定：
ip -details link show can0
ip -details link show can1
應確認介面處於 UP 狀態，並檢查 CAN FD 與 bitrate 設定。由於此專案的容器使用 Host Network 且具有必要的裝置權限，CAN 介面通常可以在容器中存取；本文件建議在主機端先完成設定。
5.3 啟動 OpenArm 實體雙臂
在啟動前，確認機器人周圍無人員與障礙物、緊急停止可用、供電與 CAN 線路正常，且已核對機器人版本。切勿將 Fake Hardware 的成功啟動視為實機安全驗證通過。

在 Docker 容器內 執行：
source /root/openarm/environment.sh 0
ros2 launch openarm_bringup openarm.bimanual.launch.py arm_type:=v10 use_fake_hardware:=false
其中：
參數	說明
arm_type:=v10	指定 OpenArm 版本（請與實際硬體核對）
use_fake_hardware:=true	模擬硬體，不連接真實馬達
use_fake_hardware:=false	使用真實硬體介面，可能實際驅動馬達


安全提醒： 本指令可能啟動實際硬體控制；首次測試時應由熟悉設備的人員在場，避免直接發送未驗證的關節位置命令。
6. 確認 ROS 2 與硬體狀態
啟動 Bringup 後，另開一個終端機進入同一容器：
docker exec -it openarm-dev bash
source /root/openarm/environment.sh 0
接著檢查：
# 檢查 ROS 2 節點
ros2 node list

# 檢查控制器狀態
ros2 control list_controllers

# 檢查關節狀態 Topic
ros2 topic list | grep joint_states
ros2 topic echo /joint_states --once
實機模式還可以在主機執行：
ip -details -statistics link show can0
ip -details -statistics link show can1
驗證重點： 確認預期的節點與控制器存在、控制器狀態正確、關節資料有持續更新，且 CAN 介面沒有持續累積異常錯誤。節點出現不代表硬體已安全就緒。
7. 常見問題
問題	檢查方向
can0 或 can1 不存在	在主機檢查 CAN 轉接器、驅動、連接線與介面名稱
CAN 介面無法設為 UP	檢查是否支援 CAN FD、bitrate、介面狀態及系統錯誤訊息
ros2 node list 看不到預期節點	確認 Bringup 仍在執行、`ROS_DOMAIN_ID` 一致，並檢查 ROS 2 啟動錯誤
出現 CycloneDDS 網卡名稱錯誤	檢查 CYCLONEDDS_URI 是否指定不存在的網卡；若是舊設定造成問題，可在目前終端機 unset CYCLONEDDS_URI 後重新測試
RViz2 沒有顯示	檢查主機的 X11、`DISPLAY`、X11 權限與是否有啟動 RViz2
編譯失敗	查看 colcon 的第一個錯誤；首次編譯使用 colcon_build.sh 解決 openarm_can 依賴順序


8. 專案主要檔案
路徑	用途
run.sh	建立／啟動 Docker 容器並進入 Bash
docker/docker-compose.openarm.yml	Docker 容器、Host Network、硬體與 X11 掛載設定
docker/dockerfile.openarm	ROS 2 Humble 及相關依賴環境
colcon_build.sh	ROS 2 工作區初次完整編譯
environment.sh	載入 ROS 2 工作區並設定 ROS_DOMAIN_ID
ros2_ws/src/	OpenArm ROS 2 套件原始碼
openarm_urdf/	機器人模型相關檔案


9. 關閉
在執行 ros2 launch 的終端機使用 Ctrl+C 正常結束 Bringup；實機操作時，仍應依設備的安全停機程序確認馬達與機械臂狀態。

若要在 Ubuntu 主機 關閉 Docker 開發容器：
docker compose -f docker/docker-compose.openarm.yml down
注意： 這份文件說明專案現有的 Docker／ROS 2 啟動方式；它不取代 OpenArm 硬體的完整安全操作與校正流程。
