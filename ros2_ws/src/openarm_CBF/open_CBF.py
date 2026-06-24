#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import math
import numpy as np
import socket
import struct
import cvxpy as cp
import warnings
import threading
import pybullet as p
import pybullet_data

warnings.filterwarnings("ignore", category=UserWarning, module="cvxpy")

# ================= ⚙️ CBF 參數設定 =================
SAFETY_MARGIN = 0.20  # 警告區外邊界
MAX_CBF_POINTS = 8
SOCKET_PATH = "/tmp/koch_vamp_server.sock"
MAX_JOINT_VELOCITY = 1.5

class VAMPClient:
    """VAMP Socket Client for point cloud data"""
    def __init__(self, socket_path=SOCKET_PATH):
        self.socket_path = socket_path
    
    def get_vamp_data(self, current_joints):
        try:
            n = len(current_joints)
            sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            sock.settimeout(0.1)
            sock.connect(self.socket_path)
            # 動態產生 struct 格式，例如 7軸就是 '7f7fIf'
            fmt = f"{n}f{n}fIf" 
            req = struct.pack(fmt, *current_joints, *current_joints, 0, 0.02)
            sock.sendall(req)
            header = self._recv_exact(sock, 20)
            if not header:
                sock.close()
                return None
            success, p_size, cloud_size, sph_count, ms = struct.unpack("B3xIIII", header)
            if p_size > 0:
                self._recv_exact(sock, p_size * 24)
            if cloud_size > 0:
                raw_cloud = self._recv_exact(sock, cloud_size * 12)
                points = np.frombuffer(raw_cloud, dtype=np.float32).reshape(-1, 3).copy()
                sock.close()
                return points
            sock.close()
            return np.array([])
        except:
            return None
    
    def _recv_exact(self, sock, n):
        data = b''
        while len(data) < n:
            chunk = sock.recv(n - len(data))
            if not chunk: return None
            data += chunk
        return data

class UnityAdapterNodeWithCBF(Node):
    def __init__(self):
        super().__init__('unity_adapter_with_cbf_node')

        self.arms = ['left', 'right']
        
        # 🔴 CBF 相關設定
        self.use_cbf = True
        self.urdf_path = '/workspace/openarm_urdf/openarm_bimanual.urdf'
        self.pb_lock = threading.Lock()
        self.vamp = VAMPClient()

        # 關節硬限位 (J1~J7 + J8夾爪)
        self.joint_limits = {
            'j1': (math.radians(-80), math.radians(200)),
            'j2': (math.radians(-100), math.radians(100)),
            'j3': (math.radians(-90), math.radians(90)),
            'j4': (math.radians(0), math.radians(140)),
            'j5': (math.radians(-90), math.radians(90)),
            'j6': (math.radians(-45), math.radians(45)),
            'j7': (math.radians(-90), math.radians(90)),
            'j8': (0.0, 1.0) 
        }

        # 將限位轉換為 numpy array 供 QP Solver 使用 (只取前 7 個手臂關節)
        self.q_min = np.array([limits[0] for key, limits in list(self.joint_limits.items())[:7]])
        self.q_max = np.array([limits[1] for key, limits in list(self.joint_limits.items())[:7]])

        self.start_pose = [0.0, 0.0, 0.0, math.radians(90), 0.0, 0.0, 0.0, 0.0]

        # 狀態管理
        self.is_connected = {'left': False, 'right': False}
        self.last_msg_time = {arm: self.get_clock().now() for arm in self.arms}
        self.timeout_sec = 2.0

        # 位置紀錄
        self.current_positions = {arm: list(self.start_pose) for arm in self.arms}
        self.target_positions = {arm: list(self.start_pose) for arm in self.arms}
        self.real_positions = {arm: list(self.start_pose) for arm in self.arms}  # 新增：記錄實體手臂真實狀態

        self.max_vel_teleop = 2.5
        self.max_vel_failsafe = 0.5
        self.alpha = 0.3 

        self.subs = {}
        self.pubs = {}
        self.gripper_pubs = {}

        # 🔴 初始化 PyBullet 與 QP Solver (每隻手臂獨立的物理引擎狀態)
        self.robot_ids = {}
        self.qp_probs = {}
        self._init_cbf_kinematics_and_solver()

        for arm in self.arms:
            self.subs[arm] = self.create_subscription(
                JointState, f'/{arm}_joint_states/vr_control',
                lambda msg, a=arm: self.vr_command_callback(msg, a), 10)

            self.pubs[arm] = self.create_publisher(
                Float64MultiArray, f'/{arm}_forward_position_controller/commands', 10)
            
            self.gripper_pubs[arm] = self.create_publisher(
                Float64MultiArray, f'/{arm}_gripper_controller/commands', 10)

        # 單一訂閱整個系統的真實 joint states
        self.create_subscription(
            JointState, '/joint_states',
            self.real_state_callback, 10)

        self.last_loop_time = self.get_clock().now()
        self.control_timer = self.create_timer(0.02, self.control_loop)

        self.get_logger().info("🔥 Unity Adapter (With CBF) 已啟動！大腦與身體完美結合。")

    def _init_cbf_kinematics_and_solver(self):
        try:
            # 採用無頭模式 (背景運算，不開視窗，效能最高)
            p.connect(p.DIRECT)
            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            
            for arm in self.arms:
                # 為左右手分別載入 URDF (如果在同一個空間，可以給予不同的 basePosition)
                base_pos = [0, 0.5, 0] if arm == 'left' else [0, -0.5, 0]
                robot_id = p.loadURDF(self.urdf_path, basePosition=base_pos, useFixedBase=True)
                self.robot_ids[arm] = robot_id
                
            self.joint_indices = [i for i in range(p.getNumJoints(self.robot_ids[self.arms[0]])) 
                                  if p.getJointInfo(self.robot_ids[self.arms[0]], i)[2] != p.JOINT_FIXED]
            
            self.cbf_dof = len(self.joint_indices)
            self.get_logger().info(f"✅ PyBullet loaded. CBF Active Joints: {self.cbf_dof}")

            # 初始化左右手的 QP Solver
            for arm in self.arms:
                self.qp_probs[arm] = self._create_qp_solver()

            self.pybullet_ready = True
        except Exception as e:
            self.get_logger().error(f"❌ PyBullet 初始化失敗，CBF 將被停用: {e}")
            self.pybullet_ready = False
            self.use_cbf = False

    def _create_qp_solver(self):
        """建立獨立的 QP Solver 實例"""
        solver_dict = {}
        dof = self.cbf_dof
        
        solver_dict['dq_var'] = cp.Variable(dof)
        solver_dict['slack_var'] = cp.Variable(MAX_CBF_POINTS)
        solver_dict['P_param'] = cp.Parameter((dof, dof), PSD=True)
        solver_dict['q_param'] = cp.Parameter(dof)
        solver_dict['dq_min_param'] = cp.Parameter(dof)
        solver_dict['dq_max_param'] = cp.Parameter(dof)
        solver_dict['G_param'] = cp.Parameter((MAX_CBF_POINTS, dof))
        solver_dict['h_alpha_param'] = cp.Parameter(MAX_CBF_POINTS)
        
        # 預設值
        solver_dict['P_param'].value = np.eye(dof)
        solver_dict['q_param'].value = np.zeros(dof)
        solver_dict['dq_min_param'].value = -np.ones(dof) * MAX_JOINT_VELOCITY
        solver_dict['dq_max_param'].value = np.ones(dof) * MAX_JOINT_VELOCITY
        solver_dict['G_param'].value = np.zeros((MAX_CBF_POINTS, dof))
        solver_dict['h_alpha_param'].value = np.ones(MAX_CBF_POINTS)
        
        obj = cp.Minimize(
            0.5 * cp.quad_form(solver_dict['dq_var'], solver_dict['P_param']) + 
            solver_dict['q_param'].T @ solver_dict['dq_var'] + 
            10000.0 * cp.sum_squares(solver_dict['slack_var'])
        )
        constraints = [
            solver_dict['dq_var'] >= solver_dict['dq_min_param'],
            solver_dict['dq_var'] <= solver_dict['dq_max_param'],
            -solver_dict['G_param'] @ solver_dict['dq_var'] <= solver_dict['h_alpha_param'] + solver_dict['slack_var'],
            solver_dict['slack_var'] >= 0
        ]
        solver_dict['prob'] = cp.Problem(obj, constraints)
        return solver_dict

    def real_state_callback(self, msg: JointState):
        """接收實體手臂真實的 joint states，更新至 real_positions"""
        new_real_left = list(self.real_positions['left'])
        new_real_right = list(self.real_positions['right'])
        
        updated_left = False
        updated_right = False
        
        for i, name in enumerate(msg.name):
            name_lower = name.lower()
            target_idx = -1
            
            for idx in range(1, 8):
                if f'joint{idx}' in name_lower or f'j{idx}' in name_lower or f'link{idx}' in name_lower:
                    target_idx = idx - 1
                    break
            
            if 'finger' in name_lower:
                target_idx = 7 
            
            if target_idx != -1:
                if 'left' in name_lower:
                    new_real_left[target_idx] = msg.position[i]
                    updated_left = True
                elif 'right' in name_lower:
                    new_real_right[target_idx] = msg.position[i]
                    updated_right = True
        
        if updated_left:
            self.real_positions['left'] = new_real_left
        if updated_right:
            self.real_positions['right'] = new_real_right

    def vr_command_callback(self, msg: JointState, arm: str):
        self.last_msg_time[arm] = self.get_clock().now()
        if not self.is_connected[arm]:
            self.is_connected[arm] = True

        new_target = list(self.target_positions[arm])
        valid_count = 0

        for i, name in enumerate(msg.name):
            name_lower = name.lower()
            target_idx = -1
            
            for idx in range(1, 8):
                if f'joint{idx}' in name_lower or f'j{idx}' in name_lower or f'link{idx}' in name_lower:
                    target_idx = idx - 1
                    break
            
            if 'finger' in name_lower:
                target_idx = 7 
            
            if target_idx != -1:
                raw_pos = msg.position[i]
                limit_key = f'j{target_idx + 1}'
                min_lim, max_lim = self.joint_limits.get(limit_key, (-3.14, 3.14))
                new_target[target_idx] = max(min_lim, min(raw_pos, max_lim))
                valid_count += 1

        if valid_count > 0:
            self.target_positions[arm] = new_target

    def _compute_safe_velocity(self, arm, q_cmd, q_target, dt):
        """核心 CBF 計算：使用 PyBullet 與 VAMP Server"""
        dof = self.cbf_dof
        q_diff = q_target - q_cmd
        Kp = 5.0
        dq_desired = np.clip(Kp * q_diff, -MAX_JOINT_VELOCITY, MAX_JOINT_VELOCITY)
        
        robot_id = self.robot_ids[arm]
        
        with self.pb_lock:
            for i, idx in enumerate(self.joint_indices):
                p.resetJointState(robot_id, idx, q_cmd[i])
            arm_pts = np.array([p.getLinkState(robot_id, i)[0] for i in self.joint_indices])

        cloud = self.vamp.get_vamp_data(q_cmd) if self.vamp else None
        
        G = np.zeros((MAX_CBF_POINTS, dof))
        h = np.ones(MAX_CBF_POINTS) * 100.0
        
        if cloud is not None and len(cloud) > 0:
            pts = np.array(cloud)
            dists = np.linalg.norm(pts[:, np.newaxis, :] - arm_pts[np.newaxis, :, :], axis=2)
            min_dists = np.min(dists, axis=1)
            
            valid_indices = np.where(min_dists >= 0.10)[0]
            
            dangerous_pts = []
            if len(valid_indices) > 0:
                valid_dists = min_dists[valid_indices]
                k = min(MAX_CBF_POINTS, len(valid_dists))
                idx = np.argpartition(valid_dists, k-1)[:k]
                for i in idx:
                    dangerous_pts.append({'position': pts[valid_indices[i]]})

            while len(dangerous_pts) < MAX_CBF_POINTS:
                dangerous_pts.append({'position': np.array([10.0, 10.0, 10.0])})

            for i, obs in enumerate(dangerous_pts):
                if obs['position'][0] > 5.0 or obs['position'][2] < 0.05: continue
                
                gripper_pos = arm_pts[-1]
                if np.linalg.norm(obs['position'] - gripper_pos) < 0.15: continue

                dists_to_links = [np.linalg.norm(pt - obs['position']) for pt in arm_pts]
                near_link_array_idx = np.argmin(dists_to_links)
                true_min_dist = dists_to_links[near_link_array_idx]

                if true_min_dist < SAFETY_MARGIN:
                    near_link_idx = self.joint_indices[near_link_array_idx]
                    
                    with self.pb_lock:
                        jt, _ = p.calculateJacobian(robot_id, near_link_idx, [0,0,0], list(q_cmd), [0.0]*dof, [0.0]*dof)
                        J_obs = np.array(jt)[:, :dof]
                        vec_repulse = (arm_pts[near_link_array_idx] - obs['position'])
                    
                    unit_vec = vec_repulse / (np.linalg.norm(vec_repulse) + 1e-6)
                    signs = np.array([-1.0] * dof) # 可根據硬體旋轉方向調整
                    ratio = np.clip((SAFETY_MARGIN - true_min_dist) / SAFETY_MARGIN, 0.0, 1.0)
                    h[i] = -2.0 * (ratio ** 2)
                    G[i] = (J_obs * signs).T @ unit_vec

        # 送入 QP Solver
        solver = self.qp_probs[arm]
        solver['P_param'].value = 2.0 * np.eye(dof) + 1e-3 * np.eye(dof)
        solver['q_param'].value = -2.0 * dq_desired
        
        q_min_dof = self.q_min[:dof] if len(self.q_min) >= dof else -np.ones(dof) * 3.14
        q_max_dof = self.q_max[:dof] if len(self.q_max) >= dof else np.ones(dof) * 3.14
        
        solver['dq_min_param'].value = np.maximum((q_min_dof - q_cmd) / dt, -MAX_JOINT_VELOCITY)
        solver['dq_max_param'].value = np.minimum((q_max_dof - q_cmd) / dt, MAX_JOINT_VELOCITY)
        solver['G_param'].value = G
        solver['h_alpha_param'].value = h
        
        try:
            solver['prob'].solve(solver=cp.OSQP, verbose=False, warm_start=False)
            return solver['dq_var'].value if solver['dq_var'].value is not None else dq_desired
        except:
            return dq_desired

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

            # 🔴 分離控制：1. 處理手臂關節 (透過 CBF)
            dof = self.cbf_dof if hasattr(self, 'cbf_dof') else 7
            arm_target = np.array(self.target_positions[arm][:dof])
            arm_current = np.array(self.real_positions[arm][:dof])  # 改為使用實體的真實狀態

            if self.use_cbf and self.pybullet_ready and self.is_connected[arm]:
                # 執行 CBF 防撞運算
                dq_safe = self._compute_safe_velocity(arm, arm_current, arm_target, dt)
                arm_next = arm_current + dq_safe * dt
                # 確保不超出硬體極限
                arm_next = np.clip(arm_next, self.q_min[:dof], self.q_max[:dof])
                temp_full_pos.extend(arm_next.tolist())
            else:
                # 傳統低通濾波與限速控制
                for i in range(dof):
                    target = arm_target[i]
                    current = arm_current[i]
                    smoothed = self.alpha * target + (1.0 - self.alpha) * current if self.is_connected[arm] else target
                    diff = smoothed - current
                    step = max(-max_step, min(diff, max_step))
                    temp_full_pos.append(current + step)

            # 🔴 分離控制：2. 處理夾爪 (純 Teleop，不參與避障)
            for i in range(dof, 8):
                target = self.target_positions[arm][i]
                current = self.real_positions[arm][i]  # 改為使用實體的真實狀態
                smoothed = self.alpha * target + (1.0 - self.alpha) * current if self.is_connected[arm] else target
                diff = smoothed - current
                step = max(-max_step, min(diff, max_step))
                temp_full_pos.append(current + step)

            self.current_positions[arm] = temp_full_pos

            # 發送給 ROS2 Controller
            arm_msg = Float64MultiArray(data=temp_full_pos[:7])
            self.pubs[arm].publish(arm_msg)

            gripper_msg = Float64MultiArray(data=[temp_full_pos[7]])
            self.gripper_pubs[arm].publish(gripper_msg)

def main(args=None):
    rclpy.init(args=args)
    node = UnityAdapterNodeWithCBF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        if hasattr(node, 'pybullet_ready') and node.pybullet_ready:
            p.disconnect()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()