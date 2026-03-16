#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
from builtin_interfaces.msg import Duration
import math

class UnityAdapterNode(Node):
    def __init__(self):
        super().__init__('unity_adapter_node')
        
        # 定義雙臂名稱
        self.arms = ['left', 'right']
        
        # OpenArm 各關節的硬體限位 (依照官方文件規格轉換為 rad)
        # 參考圖片:
        # J1: -80 to +200 deg
        # J2: -100 to +100 deg
        # J3: -90 to +90 deg
        # J4: 0 to +140 deg
        # J5: -90 to +90 deg
        # J6: -45 to +45 deg
        # J7: -90 to +90 deg
        self.joint_limits = {
            'j1': (math.radians(-80), math.radians(200)),
            'j2': (math.radians(-100), math.radians(100)),
            'j3': (math.radians(-90), math.radians(90)),
            'j4': (math.radians(0), math.radians(140)),
            'j5': (math.radians(-90), math.radians(90)),
            'j6': (math.radians(-45), math.radians(45)),
            'j7': (math.radians(-90), math.radians(90)),
        }
        
        # 安全預設姿態 (Start Pose)
        # 當節點啟動，或是 Watchdog 觸發安全機制時，手臂會回到此角度
        self.start_pose = {
            'j1': 0.0,
            'j2': 0.0,
            'j3': 0.0,
            'j4': math.radians(90), # 手臂些微彎曲避免剛性伸直
            'j5': 0.0,
            'j6': 0.0,
            'j7': 0.0
        }
        
        # 紀錄最後一次收到 Unity 訊號的時間與狀態
        self.last_msg_time = {'left': self.get_clock().now(), 'right': self.get_clock().now()}
        self.is_connected = {'left': False, 'right': False}
        self.timeout_sec = 0.5 # 0.5秒沒收到代表斷線
        
        # 訂閱與發布宣告
        self.subs = {}
        self.pubs = {}
        
        for arm in self.arms:
            # 1. 訂閱來自 Unity ROS# 的軌跡資料
            self.subs[arm] = self.create_subscription(
                JointState,
                f'/{arm}_joint_states/vr_control',
                lambda msg, arm_name=arm: self.vr_command_callback(msg, arm_name),
                10
            )
            
            # 2. 轉發給 Follower ros2_control
            self.pubs[arm] = self.create_publisher(
                JointTrajectory,
                f'/{arm}_joint_trajectory_controller/joint_trajectory',
                10
            )
            
            # 第一個動作：送出 Start Pose 給機器人確保安全
            self.send_start_pose(arm)

        # 3. Watchdog Timer：定期檢查 Unity 是不是斷線了
        self.watchdog_timer = self.create_timer(0.1, self.watchdog_check)
        
        self.get_logger().info("🔥 Unity Adapter Node 已經啟動！雙臂處於 Start Pose。")

    def vr_command_callback(self, msg: JointState, arm: str):
        # 收到 Unity 的封包，更新 watchdog 時間
        self.last_msg_time[arm] = self.get_clock().now()
        
        if not self.is_connected[arm]:
            self.get_logger().info(f"✅ {arm.capitalize()} 臂已連接到 Unity VR 控制！")
            self.is_connected[arm] = True
            
        safe_positions = []
        joint_names = []
        
        # 抓取並驗證收到的每個關節
        for i, name in enumerate(msg.name):
            target_j = None
            for j_key in ['j1', 'j2', 'j3', 'j4', 'j5', 'j6', 'j7']:
                if j_key in name.lower():
                    target_j = j_key
                    break
                    
            if not target_j:
                continue
                
            raw_pos = msg.position[i]
            
            # 執行 Safety Clamp (硬限制)
            min_lim, max_lim = self.joint_limits[target_j]
            safe_pos = max(min_lim, min(raw_pos, max_lim))
            
            safe_positions.append(safe_pos)
            
            # 將關節名稱對應給 ros2_control 負責控制的名字
            # 根據 openarm.bimanual.ros2_control.xacro 定義：
            # 名稱為: openarm_left_joint1, openarm_right_joint1
            joint_names.append(f"openarm_{arm}_joint{target_j[1]}") 

        # 組裝成 JointTrajectory 並送給 controller
        if len(safe_positions) > 0:
            self.republish_to_controller(arm, joint_names, safe_positions, duration_sec=0.1)

    def watchdog_check(self):
        now = self.get_clock().now()
        for arm in self.arms:
            if self.is_connected[arm]:
                elapsed = (now - self.last_msg_time[arm]).nanoseconds / 1e9
                if elapsed > self.timeout_sec:
                    self.get_logger().error(f"🚨 {arm.capitalize()} 臂失去 Unity 連線 ({elapsed:.1f}s)！觸發 Fail-safe 返回 Start Pose。")
                    self.is_connected[arm] = False
                    self.send_start_pose(arm)

    def send_start_pose(self, arm: str):
        # 將機器人拉回預設站姿，名稱必須與 URDF (xacro) 完全吻合
        joint_names = [f"openarm_{arm}_joint1", f"openarm_{arm}_joint2", f"openarm_{arm}_joint3", f"openarm_{arm}_joint4", f"openarm_{arm}_joint5", f"openarm_{arm}_joint6", f"openarm_{arm}_joint7"]
        positions = [self.start_pose['j1'], self.start_pose['j2'], self.start_pose['j3'], self.start_pose['j4'], self.start_pose['j5'], self.start_pose['j6'], self.start_pose['j7']]
        
        # 啟動或斷線回原本位置時，動作放慢 (3秒) 以測安全
        self.republish_to_controller(arm, joint_names, positions, duration_sec=3.0)
        self.get_logger().info(f"🔄 發送 {arm.capitalize()} 臂 Start Pose 指令...")

    def republish_to_controller(self, arm: str, joint_names: list, positions: list, duration_sec: float):
        traj_msg = JointTrajectory()
        traj_msg.header = Header()
        traj_msg.header.stamp = self.get_clock().now().to_msg()
        traj_msg.joint_names = joint_names
        
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        
        # 告訴 ros2_control 這個目標要多久到達
        sec = int(duration_sec)
        nanosec = int((duration_sec - sec) * 1e9)
        point.time_from_start = Duration(sec=sec, nanosec=nanosec)
        
        traj_msg.points.append(point)
        # 因為是連續目標，可以發送 publisher
        self.pubs[arm].publish(traj_msg)

def main(args=None):
    rclpy.init(args=args)
    node = UnityAdapterNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("程式被手動終止。")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
