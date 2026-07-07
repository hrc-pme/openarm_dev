#!/usr/bin/env python3

import pybullet as p
import pybullet_data

# 1. 匯入 ROS2 相關函式庫
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from ament_index_python.packages import get_package_share_directory
import numpy as np
import os

# --- Configuration ---
# Path to your URDF file, this is the path inside the docker container
URDF_PATH = "/root/openarm/openarm_urdf/openarm_bimanual.urdf"


class UrdfTestNode(Node):
    """
    一個 ROS2 節點，用於在 PyBullet 中測試 URDF 載入，並即時視覺化真實手臂的關節狀態。
    """
    def __init__(self):
        super().__init__('urdf_test_node')
        self.robot_id = None
        self.joint_map = {}
        self.current_joint_states = {}
        # 🔴 依照您的要求，將手臂直接放置在地板上 (z=0)
        self.base_position = [0, 0, 0]
        self._is_initialized = False # Flag for successful initialization

        # 🔴 新增：用於點雲視覺化的變數
        self.filtered_points = None
        self.point_cloud_visual_id = None

        self.get_logger().info("🚀 啟動 PyBullet URDF 測試並訂閱 ROS2 Topic...")

        # 2. 連接到物理伺服器
        try:
            p.connect(p.GUI)
            self.get_logger().info("✅ 已連接到 PyBullet (GUI 模式)。")
        except p.error as e:
            self.get_logger().error(f"❌ 連接 PyBullet 失敗: {e}")
            self.get_logger().error("   請確認您已配置顯示環境 (例如 X11)。")
            rclpy.shutdown()
            return

        # 3. 設定模擬環境
        # 終極解決方案：只設定一次我們自己模型的搜尋路徑，
        # 然後使用絕對路徑載入 PyBullet 的預設模型，以避免路徑衝突。
        self.get_logger().info("設定機器人模型的 Mesh 搜尋路徑...")
        try:
            # URDF 中的 mesh 路徑是 'openarm_description/meshes/...'，這意味著
            # PyBullet 的搜尋路徑必須是包含 'openarm_description' 資料夾的那個目錄，
            # 在 colcon build 的環境中，這個目錄就是 'install'。
            openarm_description_share = get_package_share_directory('openarm_description')
            # get_package_share_directory -> .../install/openarm_description/share/openarm_description
            # 我們需要的是 '.../install' 這個根目錄
            install_path = os.path.dirname(os.path.dirname(os.path.dirname(openarm_description_share)))
            p.setAdditionalSearchPath(install_path)
            self.get_logger().info(f"✅ 已設定 Mesh 搜尋路徑: {install_path}")
        except Exception as e:
            self.get_logger().warn(f"⚠️  找不到 'openarm_description' 套件，URDF 模型可能無法正確載入 mesh: {e}")

        # 設定重力並使用絕對路徑載入地板
        p.setGravity(0, 0, -9.81)
        plane_path = os.path.join(pybullet_data.getDataPath(), "plane.urdf")
        p.loadURDF(plane_path)
        self.get_logger().info("✅ 已載入地板模型。")

        # 🔴 依照您的要求，將相機視角設定為完全的俯視
        p.resetDebugVisualizerCamera(
            cameraDistance=2.0,            # 視角距離 (之前 35.0 是打字錯誤)
            cameraYaw=0,                   # 沿著 X 軸方向看
            cameraPitch=-89.9,             # 幾乎是垂直往下看 (-90度會鎖死視角)
            cameraTargetPosition=[0, 0, 0.5] # 鏡頭對準手臂中心
        )

        # 4. 處理 URDF：將 package:// 轉為絕對路徑 (解決 PyBullet 不認得 ROS 路徑的問題)
        self.get_logger().info(f"正在處理 URDF: {URDF_PATH}")
        try:
            # 動態找到 mesh 檔案所在的 ROS package 的絕對路徑，避免寫死路徑
            mesh_package_path = get_package_share_directory('openarm_description')

            with open(URDF_PATH, 'r') as f:
                urdf_content = f.read()
            # 替換掉無法識別的路徑格式
            modified_urdf = urdf_content.replace("package://openarm_description", mesh_package_path)
            temp_urdf_path = "/tmp/modified_openarm.urdf"
            with open(temp_urdf_path, 'w') as f:
                f.write(modified_urdf)
            
            # 從暫存檔案載入 URDF
            self.robot_id = p.loadURDF(temp_urdf_path, basePosition=self.base_position, useFixedBase=True)
            self.get_logger().info(f"✅ 成功載入 URDF。機器人 ID: {self.robot_id}")
            self._build_joint_map()
            self._colorize_robot()
        except p.error as e:
            self.get_logger().error(f"❌ 載入 URDF 檔案失敗: {e}")
            self.get_logger().error("   請檢查 URDF 路徑與 mesh 檔案路徑是否正確。節點將不會執行。")
            return

        # 4. 建立 ROS2 訂閱者
        self.real_state_sub = self.create_subscription(
            JointState,
            '/joint_states',  # 訂閱標準的真實關節狀態 Topic
            self.real_state_callback,
            10)
        self.get_logger().info("✅ 已訂閱 /joint_states Topic，準備視覺化真實手臂狀態。")

        # 🔴 新增：訂閱處理過的點雲
        # 🔴 關鍵修正：手動建立一個與 C++ 發布端完全匹配的 QoS Profile
        # 這可以解決因預設 QoS Profile 不匹配而導致的訂閱失敗問題
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        cbf_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.point_cloud_sub = self.create_subscription(
            PointCloud2,
            '/cbf/filtered_points',  # 訂閱 C++ server 處理後的點雲
            self.point_cloud_callback,
            cbf_qos_profile # 使用我們自訂的 QoS
        )
        self.get_logger().info("✅ 已訂閱 /cbf/filtered_points Topic，準備視覺化過濾後的點雲。")

        # 5. 建立計時器
        self.timer = self.create_timer(1.0 / 60.0, self.update_simulation) # 60 Hz 更新率
        self.get_logger().info("✅ 模擬迴圈已啟動。")
        self._is_initialized = True # Mark initialization as successful

    def is_initialized(self):
        return self._is_initialized

    def _build_joint_map(self):
        """
        檢查載入的 URDF，並建立一個從關節名稱到其 PyBullet 索引的映射。
        """
        num_joints = p.getNumJoints(self.robot_id)
        self.get_logger().info(f"   URDF 中的關節數量: {num_joints}")
        for i in range(num_joints):
            joint_info = p.getJointInfo(self.robot_id, i)
            joint_name = joint_info[1].decode('utf-8')
            self.joint_map[joint_name] = i
            self.get_logger().info(f"   - 已映射關節 '{joint_name}' 到索引 {i}")

    def _colorize_robot(self):
        """
        將機器人的視覺外觀設定為中性灰色。
        """
        p.changeVisualShape(self.robot_id, -1, rgbaColor=[0.7, 0.7, 0.7, 1.0])
        for i in range(p.getNumJoints(self.robot_id)):
            p.changeVisualShape(self.robot_id, i, rgbaColor=[0.7, 0.7, 0.7, 1.0])

    def real_state_callback(self, msg: JointState):
        """
        回呼函式，用於接收並儲存最新的真實關節狀態。
        /joint_states 中的關節名稱應直接對應 URDF 中的名稱。
        """
        # 使用 throttle 避免日誌洗版，並顯示收到的關節名稱
        self.get_logger().info(f"Real joint states received. Names: {[n for n in msg.name]}", throttle_duration_sec=5.0)
        
        match_found_in_msg = False
        for i, name in enumerate(msg.name):
            if name in self.joint_map:
                self.current_joint_states[name] = msg.position[i]
                match_found_in_msg = True
                # 使用 DEBUG 等級日誌，避免在正常運作時洗版。
                self.get_logger().debug(f"  ✅ Matched and updated '{name}' to position {msg.position[i]:.4f}")
            else:
                # 使用 WARN 等級日誌，但加上 throttle，只在剛開始或名稱變更時提示一次，避免洗版。
                self.get_logger().warn(
                    f"  ❌ Joint name '{name}' from /joint_states not found in URDF's joint map. Please check for naming mismatches.",
                    throttle_duration_sec=10.0, once=True
                )
        
        if not match_found_in_msg:
            self.get_logger().warn("In the last message from /joint_states, NO joint names matched the URDF. Please check the full list of names.", throttle_duration_sec=5.0)

    def point_cloud_callback(self, msg: PointCloud2):
        """
        回呼函式，接收並儲存過濾後的點雲資料。
        """
        self.get_logger().info(f"📥 成功接收到原始點雲訊息！大小: {msg.width * msg.height} 點", throttle_duration_sec=2.0)

        try:
            points_structured = pc2.read_points_numpy(msg, field_names=("x", "y", "z"))
            
            if points_structured.size == 0:
                self.filtered_points = []
                self.get_logger().info("⚠️ 收到的點雲是空的", throttle_duration_sec=2.0)
                return
                
            points = np.array(points_structured.tolist())
            
            # 防呆：確保維度正確
            if points.ndim != 2 or points.shape[1] != 3:
                self.filtered_points = []
                return

            # 過濾無效的 NaN / Inf 數值
            valid_mask = np.isfinite(points).all(axis=1)
            points = points[valid_mask]
            
            if len(points) == 0:
                self.filtered_points = []
                return

            # 🔴 核心修正：移除 Python 端的座標轉換！
            # 因為 C++ Server 已經轉好變成 Robot Base 座標系了，
            # 這裡我們直接使用 C++ 傳來的 `points`，不再做加減乘除。
            
            # 轉回純 Python List，PyBullet 必須吃這個格式
            self.filtered_points = points.tolist()
            
            # 印出第一個點的真實座標，幫你確認點雲有沒有在手臂前方
            if self.filtered_points:
                pt = self.filtered_points[0]
                self.get_logger().info(f"✅ 成功處理 {len(self.filtered_points)} 個點。首點座標: [{pt[0]:.2f}, {pt[1]:.2f}, {pt[2]:.2f}]", throttle_duration_sec=2.0)

        except Exception as e:
            self.get_logger().warn(f"處理點雲時發生錯誤: {e}", throttle_duration_sec=5.0)

    def update_simulation(self):
        """
        計時器回呼，用最新的關節狀態和點雲更新 PyBullet 模擬。
        """
        if self.robot_id is None:
            return

        # 根據接收到的數據更新關節位置
        for name, index in self.joint_map.items():
            if name in self.current_joint_states:
                position = self.current_joint_states[name]
                # 使用 resetJointState 直接控制視覺化中的關節
                p.resetJointState(self.robot_id, index, targetValue=position)

        # 🔴 視覺化點雲
        # 首先移除上一幀的點
        if self.point_cloud_visual_id is not None:
            p.removeUserDebugItem(self.point_cloud_visual_id)
            self.point_cloud_visual_id = None

        # 如果有新的點，則繪製它們
        if self.filtered_points is not None and len(self.filtered_points) > 0:
            # 依照您的要求，不對點雲高度進行調整，直接視覺化
            self.point_cloud_visual_id = p.addUserDebugPoints(
                pointPositions=self.filtered_points,
                pointColorsRGB=[[1, 0, 0]] * len(self.filtered_points), # 將點設為紅色
                pointSize=3.0 # 調小一點避免畫面太亂
            )

        # 推進模擬
        p.stepSimulation()

    def on_shutdown(self):
        """
        在節點關閉時清理資源。
        """
        self.get_logger().info("節點正在關閉，中斷與 PyBullet 的連接。")
        if p.isConnected():
            p.disconnect()


def main(args=None):
    """
    主函式，初始化 ROS2 並執行節點。
    """
    rclpy.init(args=args)
    node = None
    try:
        node = UrdfTestNode()
        if not node.is_initialized():
            raise RuntimeError("節點初始化失敗，請檢查日誌。")
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError) as e:
        if isinstance(e, RuntimeError):
            print(f"\n錯誤: {e}")
        else:
            print("\n使用者中斷模擬。")
    finally:
        # 修正：確保在節點成功初始化後才呼叫清理函式
        if node is not None:
            node.on_shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("已中斷與 PyBullet 的連接。測試完成。 👋")

if __name__ == '__main__':
    main()