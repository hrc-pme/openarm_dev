#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import math

class UnityAdapterNode(Node):
    def __init__(self):
        super().__init__('unity_adapter_node')

        self.arms = ['left', 'right']

        # 關節硬限位 (J1~J7 + J8夾爪)
        self.joint_limits = {
            'j1': (math.radians(-80), math.radians(200)),
            'j2': (math.radians(-100), math.radians(100)),
            'j3': (math.radians(-90), math.radians(90)),
            'j4': (math.radians(0), math.radians(140)),
            'j5': (math.radians(-90), math.radians(90)),
            'j6': (math.radians(-45), math.radians(45)),
            'j7': (math.radians(-90), math.radians(90)),
            'j8': (0.0, 1.0) # 假設夾爪限位是 0 到 1.0
        }

        # Start Pose (長度改為 8，最後一個是夾爪)
        self.start_pose = [0.0, 0.0, 0.0, math.radians(90), 0.0, 0.0, 0.0, 0.0]

        # 狀態管理
        self.is_connected = {'left': False, 'right': False}
        self.last_msg_time = {arm: self.get_clock().now() for arm in self.arms}
        self.timeout_sec = 2.0

        # 位置紀錄 (長度皆改為 8)
        self.current_positions = {arm: list(self.start_pose) for arm in self.arms}
        self.target_positions = {arm: list(self.start_pose) for arm in self.arms}

        self.max_vel_teleop = 2.5
        self.max_vel_failsafe = 0.5
        self.alpha = 0.3 

        self.subs = {}
        self.pubs = {}
        self.gripper_pubs = {}

        for arm in self.arms:
            # 1. 接收手臂+夾爪數據
            self.subs[arm] = self.create_subscription(
                JointState, f'/{arm}_joint_states/vr_control',
                lambda msg, a=arm: self.vr_command_callback(msg, a), 10)

            # 2. 發布給手臂控制器
            self.pubs[arm] = self.create_publisher(
                Float64MultiArray, f'/{arm}_forward_position_controller/commands', 10)
            
            # 3. 發布給夾爪控制器 (對應你的 YAML 名稱)
            self.gripper_pubs[arm] = self.create_publisher(
                Float64MultiArray, f'/{arm}_gripper_controller/commands', 10)

        self.last_loop_time = self.get_clock().now()
        self.control_timer = self.create_timer(0.02, self.control_loop)

        self.get_logger().info("🔥 Unity Adapter 已修復！支援手臂+夾爪雙控。")

    def vr_command_callback(self, msg: JointState, arm: str):
        self.last_msg_time[arm] = self.get_clock().now()
        if not self.is_connected[arm]:
            self.is_connected[arm] = True

        new_target = list(self.target_positions[arm])
        valid_count = 0

        # --- 修正後的邏輯：必須在 callback 內處理 msg ---
        for i, name in enumerate(msg.name):
            name_lower = name.lower()
            target_idx = -1
            
            # 匹配手臂 J1~J7
            for idx in range(1, 8):
                if f'joint{idx}' in name_lower or f'j{idx}' in name_lower or f'link{idx}' in name_lower:
                    target_idx = idx - 1
                    break
            
            # 匹配夾爪 (finger)
            if 'finger' in name_lower:
                target_idx = 7 # 放入第 8 個位置
            
            if target_idx != -1:
                raw_pos = msg.position[i]
                # 取得對應限位
                limit_key = f'j{target_idx + 1}'
                min_lim, max_lim = self.joint_limits.get(limit_key, (-3.14, 3.14))
                new_target[target_idx] = max(min_lim, min(raw_pos, max_lim))
                valid_count += 1

        if valid_count > 0:
            self.target_positions[arm] = new_target

    def control_loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_loop_time).nanoseconds / 1e9
        self.last_loop_time = now

        for arm in self.arms:
            elapsed_vr = (now - self.last_msg_time[arm]).nanoseconds / 1e9
            if self.is_connected[arm] and elapsed_vr > self.timeout_sec:
                self.is_connected[arm] = False
                self.target_positions[arm] = list(self.start_pose)

            max_step = (self.max_vel_teleop if self.is_connected[arm] else self.max_vel_failsafe) * dt

            temp_full_pos = []
            for i in range(8): # 遍歷 8 個關節
                target = self.target_positions[arm][i]
                current = self.current_positions[arm][i]
                
                smoothed = self.alpha * target + (1.0 - self.alpha) * current if self.is_connected[arm] else target
                diff = smoothed - current
                step = max(-max_step, min(diff, max_step))
                temp_full_pos.append(current + step)

            self.current_positions[arm] = temp_full_pos

            # --- 分流發送 ---
            # 1. 發送前 7 個給手臂
            arm_msg = Float64MultiArray(data=temp_full_pos[:7])
            self.pubs[arm].publish(arm_msg)

            # 2. 發送第 8 個給夾爪 (格式為 [pos])
            gripper_msg = Float64MultiArray(data=[temp_full_pos[7]])
            self.gripper_pubs[arm].publish(gripper_msg)

def main(args=None):
    rclpy.init(args=args)
    node = UnityAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()