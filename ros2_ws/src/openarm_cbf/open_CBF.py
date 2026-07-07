#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64MultiArray # type: ignore
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math
import numpy as np
import cvxpy as cp
import warnings
import threading
import pybullet as p
import pybullet_data
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

warnings.filterwarnings("ignore", category=UserWarning, module="cvxpy")

# ================= ⚙️ CBF 參數設定 =================
SAFETY_MARGIN = 0.35  # 警告區外邊界。必須大於 SELF_FILTER_RADIUS。
SELF_FILTER_RADIUS = 0.15 # 手臂自身半徑，小於此距離的點雲將被忽略
CBF_GAMMA = 8.0       # CBF 斥力係數。越大煞車越硬、反應越靈敏。
CBF_REPULSIVE_GAIN = 1.5 # CBF 推力係數。越大「逃跑」的感覺越強烈，產生主動推力。
MAX_CBF_POINTS = 8
MAX_JOINT_VELOCITY = 0.8

class UnityAdapterNodeWithCBF(Node):
    def __init__(self):
        super().__init__('unity_adapter_with_cbf_node')

        self.arms = ['left', 'right']
        
        # 🔴 CBF 相關設定
        self.use_cbf = True
        # The path should match the location inside the Docker container.
        self.urdf_path = '/root/openarm/openarm_urdf/openarm_bimanual.urdf'
        self.point_cloud_topic = '/cbf/filtered_points' # 🔴 訂閱處理過的點雲 Topic

        self.point_cloud = None
        self.pc_lock = threading.Lock()
        self.pb_lock = threading.Lock()

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
        self.robot_id = None
        self.qp_probs = {}
        self.joint_indices = {}
        self.joint_name_to_control_idx = {} # 🔴 新增：從 URDF 關節名稱到控制索引的映射
        # 完整雙臂 self-filter 幾何：包含旋轉、固定關節與夾爪 link。
        self.self_filter_links = {'left': [], 'right': []}
        self._init_cbf_kinematics_and_solver()

        for arm in self.arms:
            self.subs[arm] = self.create_subscription(
                JointState, f'/{arm}_joint_states/vr_control',
                lambda msg, a=arm: self.vr_command_callback(msg, a), 10)

            self.pubs[arm] = self.create_publisher(
                Float64MultiArray, f'/{arm}_forward_position_controller/commands', 10
            )
            
            self.gripper_pubs[arm] = self.create_publisher(
                Float64MultiArray, f'/{arm}_gripper_controller/commands', 10)

        # 單一訂閱整個系統的真實 joint states
        self.create_subscription(
            JointState, '/joint_states',
            self.real_state_callback, 10)

        # 🔴 新增：訂閱處理過的點雲
        # 🔴 關鍵修正：手動建立一個與 C++ 發布端完全匹配的 QoS Profile
        cbf_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.create_subscription(
            PointCloud2, self.point_cloud_topic,
            self.point_cloud_callback,
            cbf_qos_profile)

        self.last_loop_time = self.get_clock().now()
        self.control_timer = self.create_timer(0.02, self.control_loop)

        # 🔴 新增：用於點雲視覺化的變數
        self.point_cloud_visual_id = None

        # 🔴 新增：初始化狀態旗標，用於等待真實手臂位置
        self.initial_state_received = False
        self.initialization_start_time = self.get_clock().now()
        self.initialization_timeout = 5.0 # 5秒後無論如何都開始

        self.get_logger().info("🔥 Unity Adapter (With CBF) 已啟動！大腦與身體完美結合。")

    def _init_cbf_kinematics_and_solver(self):
        try:
            # 🔴 修正：將 p.DIRECT 改為 p.GUI 以啟用視覺化介面。
            # 這將會開啟一個 PyBullet 視窗，顯示機器人和點雲。
            p.connect(p.GUI) # 使用 p.DIRECT 則為無頭模式
            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            
            # 載入地板與單一機器人模型
            p.loadURDF("plane.urdf")
            # 🔴 依照您的要求，將手臂直接放置在地板上 (z=0)
            self.base_pos = [0, 0, 0]
            self.robot_id = p.loadURDF(self.urdf_path, basePosition=self.base_pos, useFixedBase=True)

            # 🔴 新增：設定攝影機視角，與 test_urdf.py 一致
            p.resetDebugVisualizerCamera(
                cameraDistance=2.0,
                cameraYaw=0,
                cameraPitch=-89.9,
                cameraTargetPosition=[0, 0, 0.5]
            )

            # 從單一模型中解析出左右臂的關節索引
            # 🔴 新增：初始化用於儲存完整機器人狀態的變數
            self.movable_joints = []
            self.pb_idx_to_control_map = {}

            num_joints_in_urdf = p.getNumJoints(self.robot_id)
            for i in range(num_joints_in_urdf):
                info = p.getJointInfo(self.robot_id, i)
                joint_name = info[1].decode('utf-8')
                joint_name_lower = joint_name.lower()
                link_name_lower = info[12].decode('utf-8').lower()

                # PyBullet 以 joint index 代表該 joint 的 child link。這裡不只收集
                # 可動關節，固定 link 與夾爪也要納入點雲 self-filter。
                filter_arm = None
                if 'left' in joint_name_lower or 'left' in link_name_lower:
                    filter_arm = 'left'
                elif 'right' in joint_name_lower or 'right' in link_name_lower:
                    filter_arm = 'right'
                if filter_arm is not None:
                    parent_link_idx = info[16]
                    self.self_filter_links[filter_arm].append((i, parent_link_idx))
                
                if info[2] != p.JOINT_FIXED:
                    self.movable_joints.append(i)

                # 1. 處理 CBF 計算所需的關節索引
                is_arm_joint = info[2] != p.JOINT_FIXED and not any(keyword in joint_name_lower for keyword in ['finger', 'gripper', 'j8'])
                
                if is_arm_joint:
                    if 'left' in joint_name_lower:
                        self.joint_indices.setdefault('left', []).append(i)
                    elif 'right' in joint_name_lower:
                        self.joint_indices.setdefault('right', []).append(i)

                # 2. 處理 ROS Topic 控制所需的關節名稱到索引的映射
                target_arm = 'left' if 'left' in joint_name_lower else 'right' if 'right' in joint_name_lower else None
                if not target_arm: continue

                target_idx = -1
                if any(keyword in joint_name_lower for keyword in ['finger', 'gripper', 'j8']):
                    target_idx = 7
                else:
                    # 🔴 修正：增加對 'jointX' 命名模式的支援，以匹配更多樣的 URDF 命名風格
                    for idx_loop in range(1, 8):
                        if f'j{idx_loop}' in joint_name_lower or f'joint{idx_loop}' in joint_name_lower:
                            target_idx = idx_loop - 1
                            break
                if target_idx != -1:
                    self.joint_name_to_control_idx[joint_name] = (target_arm, target_idx)
                    # 🔴 新增：建立從 PyBullet 關節索引到我們控制陣列的映射
                    self.pb_idx_to_control_map[i] = (target_arm, target_idx)

            # 🔴 新增：儲存總可動關節數，並建立一個從 PyBullet 索引到 Jacobian 矩陣欄位的映射
            self.num_movable_joints = len(self.movable_joints)
            self.movable_joints_map = {pb_idx: i for i, pb_idx in enumerate(self.movable_joints)}

            # 假設左右手自由度相同
            self.cbf_dof = len(self.joint_indices.get('left', []))
            if self.cbf_dof == 0 or self.cbf_dof != len(self.joint_indices.get('right', [])):
                raise Exception(f"關節索引解析失敗或左右手自由度不匹配! 左: {self.cbf_dof}, 右: {len(self.joint_indices.get('right', []))}")

            self.get_logger().info(f"✅ PyBullet loaded. CBF Active Joints per arm: {self.cbf_dof}")

            self.get_logger().info(f"✅ Built joint name to control index map. Found {len(self.joint_name_to_control_idx)} mappable joints.")
            self.get_logger().info(
                "✅ Bimanual self-filter links: "
                f"left={len(self.self_filter_links['left'])}, "
                f"right={len(self.self_filter_links['right'])}"
            )

            for pb_idx, (target_arm, control_idx) in self.pb_idx_to_control_map.items():
                if control_idx == 3:
                    p.resetJointState(self.robot_id, pb_idx, self.start_pose[3])

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

    def point_cloud_callback(self, msg: PointCloud2):
        """接收由 C++ 節點預處理過的點雲"""
        # C++ 節點已完成座標轉換、過濾和降採樣
        # Python 端只需將其轉換為 numpy 陣列即可
        try:
            # 🔴 關鍵修正：將 ROS 的點雲訊息(structured array)轉換為 CBF 計算能使用的標準 (N, 3) NumPy 陣列
            points_structured = pc2.read_points_numpy(msg, field_names=("x", "y", "z"))
            with self.pc_lock:
                if points_structured.size == 0:
                    self.point_cloud = np.array([])
                else:
                    # tolist() 是將 structured array 轉換為 list of tuples, 然後再轉為標準 ndarray
                    self.point_cloud = np.array(points_structured.tolist())
        except Exception as e:
            self.get_logger().warn(f"Failed to process point cloud: {e}", throttle_duration_sec=5.0)

    def real_state_callback(self, msg: JointState):
        """接收實體手臂真實的 joint states，更新至 real_positions"""
        # 🔴 採用更穩健的邏輯：直接使用 URDF 中的關節名稱進行匹配
        # 這與 test_urdf.py 中驗證成功的邏輯一致

        # 建立副本以避免在迭代時發生競爭條件
        new_real_left = list(self.real_positions['left'])
        new_real_right = list(self.real_positions['right'])
        
        updated_left = False
        updated_right = False
        
        for i, name in enumerate(msg.name):
            if name in self.joint_name_to_control_idx:
                arm, idx = self.joint_name_to_control_idx[name]
                position = msg.position[i]
                
                if arm == 'left' and 0 <= idx < len(new_real_left):
                    new_real_left[idx] = position
                    updated_left = True
                elif arm == 'right' and 0 <= idx < len(new_real_right):
                    new_real_right[idx] = position
                    updated_right = True

        # 如果有更新，則原子性地替換主狀態變數
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

    def _get_bimanual_self_filter_segments(self):
        """取得完整左右臂的 link 線段，作為點雲膠囊型 self-filter。"""
        starts = []
        ends = []
        base_pos = np.asarray(p.getBasePositionAndOrientation(self.robot_id)[0])

        for arm in self.arms:
            for child_idx, parent_idx in self.self_filter_links[arm]:
                child_pos = np.asarray(p.getLinkState(self.robot_id, child_idx)[0])
                if parent_idx == -1:
                    parent_pos = base_pos
                else:
                    parent_pos = np.asarray(p.getLinkState(self.robot_id, parent_idx)[0])
                starts.append(parent_pos)
                ends.append(child_pos)

        if not starts:
            return np.empty((0, 3)), np.empty((0, 3))
        return np.asarray(starts), np.asarray(ends)

    @staticmethod
    def _filter_robot_points(cloud, segment_starts, segment_ends):
        """移除左右臂任一 link 膠囊內的點雲。"""
        if cloud is None or len(cloud) == 0 or len(segment_starts) == 0:
            return cloud

        segment_vectors = segment_ends - segment_starts
        segment_lengths_sq = np.sum(segment_vectors * segment_vectors, axis=1)
        segment_lengths_sq = np.maximum(segment_lengths_sq, 1e-12)

        point_offsets = cloud[:, np.newaxis, :] - segment_starts[np.newaxis, :, :]
        projection = np.sum(
            point_offsets * segment_vectors[np.newaxis, :, :], axis=2
        ) / segment_lengths_sq[np.newaxis, :]
        projection = np.clip(projection, 0.0, 1.0)
        closest_points = (
            segment_starts[np.newaxis, :, :]
            + projection[:, :, np.newaxis] * segment_vectors[np.newaxis, :, :]
        )
        distances = np.linalg.norm(
            cloud[:, np.newaxis, :] - closest_points, axis=2
        )
        belongs_to_robot = np.min(distances, axis=1) < SELF_FILTER_RADIUS
        return cloud[~belongs_to_robot]

    def _generate_cbf_constraints(self, arm, arm_pts, cloud, q_full):
        """從點雲和手臂姿態中生成 CBF 的 G 和 h 約束矩陣"""
        dof = self.cbf_dof
        G = np.zeros((MAX_CBF_POINTS, dof))
        h = np.ones(MAX_CBF_POINTS) * 100.0 # 預設為一個非常寬鬆的約束

        if cloud is None or len(cloud) == 0:
            return G, h

        # cloud 已在 control_loop 做過一次完整雙臂 self-filter。
        dists_cloud_to_arm_links = np.linalg.norm(cloud[:, np.newaxis, :] - arm_pts[np.newaxis, :, :], axis=2)
        min_dists_to_cloud = np.min(dists_cloud_to_arm_links, axis=1)

        dangerous_pts = np.array([]) 
        if len(min_dists_to_cloud) > 0:
            k = min(MAX_CBF_POINTS, len(min_dists_to_cloud))
            nearest_indices = np.argpartition(min_dists_to_cloud, k-1)[:k]
            dangerous_pts = cloud[nearest_indices]

        # 2. 為每個威脅點建立約束
        for i, obs_pos in enumerate(dangerous_pts):
            dists_to_links = np.linalg.norm(arm_pts - obs_pos, axis=1)
            near_link_array_idx = np.argmin(dists_to_links)
            true_min_dist = dists_to_links[near_link_array_idx]

            if true_min_dist < SAFETY_MARGIN:
                near_link_idx = self.joint_indices[arm][near_link_array_idx]
                
                num_all_dof = self.num_movable_joints
                jt, _ = p.calculateJacobian(self.robot_id, near_link_idx, [0,0,0], q_full, [0.0]*num_all_dof, [0.0]*num_all_dof)
                J_obs_full = np.array(jt) 

                current_arm_pb_indices = self.joint_indices[arm]
                arm_cols_in_jacobian = [self.movable_joints_map[pb_idx] for pb_idx in current_arm_pb_indices]
                J_obs = J_obs_full[:, arm_cols_in_jacobian]

                vec_repulse = (arm_pts[near_link_array_idx] - obs_pos)
                
                # 🔴 核心修正：使用標準 CBF 煞車公式 (線性緩衝)
                gamma = 4.0  # CBF 收斂係數。越小煞車越平滑，越大煞車越硬。
                h[i] = gamma * (true_min_dist - SAFETY_MARGIN)
                # 使用 CBF 煞車公式 (h = γ * B)，其中 B 是屏障函數值 (true_min_dist - SAFETY_MARGIN)
                h[i] = CBF_GAMMA * (true_min_dist - SAFETY_MARGIN)
                G[i, :] = (J_obs.T @ (vec_repulse / (true_min_dist + 1e-6))).T
                
        return G, h

    def _compute_safe_velocity(self, arm, dq_nominal, dt, q_full, filtered_cloud):
        """核心 CBF 計算：尋找最接近 dq_nominal 且安全的速度"""
        dof = self.cbf_dof
        robot_id = self.robot_id
        current_arm_indices = self.joint_indices[arm]

        with self.pb_lock:
            arm_pts = np.array([p.getLinkState(robot_id, i)[0] for i in current_arm_indices])
            G, h = self._generate_cbf_constraints(
                arm, arm_pts, filtered_cloud, q_full
            )

        # 🔴 新增：計算一個主動「推開」的排斥速度
        # 這個速度會加到使用者意圖上，產生「逃跑」的效果
        dq_repulsive = np.zeros(dof)
        if self.use_cbf:
            for i in range(MAX_CBF_POINTS):
                # h[i] = gamma * (dist - margin). 當 dist < margin 時, h[i] 為負.
                # -h[i] 代表了障礙物侵入安全邊界的程度，我們用它來決定推力大小。
                if h[i] < 0:
                    # G[i, :] 是從障礙物指向手臂的梯度方向 (在關節空間)
                    # 我們沿著這個方向增加速度，產生推力
                    # 單位分析: (-h[i]) 是 m/s, G[i,:] 是 m/rad。為了得到 rad/s 的 dq,
                    # 我們的 GAIN 實際上隱含了複雜的單位轉換，但作為一個可調參數是有效的。
                    repulsive_force_magnitude = -h[i]
                    dq_repulsive += CBF_REPULSIVE_GAIN * repulsive_force_magnitude * G[i, :]

        # 將推力疊加到使用者的原始意圖上
        dq_nominal_with_repulsion = dq_nominal + dq_repulsive

        solver = self.qp_probs[arm]
        
        # 🔴 核心修正：將 QP 的目標設為追蹤「帶有推力的理想速度」
        # 目標函數：0.5 * ||dq - dq_nominal_with_repulsion||^2
        solver['P_param'].value = np.eye(dof)
        solver['q_param'].value = -dq_nominal_with_repulsion
        
        arm_cmd = np.array(self.current_positions[arm][:dof])
        q_min_dof = self.q_min[:dof] if len(self.q_min) >= dof else -np.ones(dof) * 3.14
        q_max_dof = self.q_max[:dof] if len(self.q_max) >= dof else np.ones(dof) * 3.14
        
        solver['dq_min_param'].value = np.maximum((q_min_dof - arm_cmd) / dt, -MAX_JOINT_VELOCITY)
        solver['dq_max_param'].value = np.minimum((q_max_dof - arm_cmd) / dt, MAX_JOINT_VELOCITY)
        solver['G_param'].value = G
        solver['h_alpha_param'].value = h
        
        try:
            solver['prob'].solve(solver=cp.OSQP, verbose=False, warm_start=True, polish=False)
            safe_dq = solver['dq_var'].value
            if safe_dq is None:
                self.get_logger().warn("CBF QP solver failed, returning nominal velocity.", throttle_duration_sec=1.0)
                return dq_nominal
            return safe_dq
        except Exception as e:
            self.get_logger().error(f"CBF QP solver threw an exception: {e}", throttle_duration_sec=1.0)
            return dq_nominal

    def control_loop(self):
        now = self.get_clock().now()

        # 🔴 新增：等待初始化的邏輯，確保 PyBullet 中的手臂位置與實體手臂同步
        if not self.initial_state_received:
            # 檢查是否已收到真實位置 (與初始的 start_pose 不同)
            # 這裡假設如果收到真實狀態，它幾乎不可能剛好等於 start_pose
            left_is_real = self.real_positions['left'] != self.start_pose
            right_is_real = self.real_positions['right'] != self.start_pose

            # 檢查是否超時
            timed_out = (now - self.initialization_start_time).nanoseconds / 1e9 > self.initialization_timeout

            if (left_is_real and right_is_real) or timed_out:
                if timed_out and not (left_is_real and right_is_real):
                    self.get_logger().warn(f"⚠️ Timed out waiting for initial /joint_states. Using default start pose.")
                else:
                    self.get_logger().info("✅ Initial joint states received. Synchronizing and starting control loop.")
                
                # 將內部追蹤的 current_positions 與真實位置同步，避免啟動時跳動
                self.current_positions['left'] = list(self.real_positions['left'])
                self.current_positions['right'] = list(self.real_positions['right'])
                
                self.initial_state_received = True
                self.last_loop_time = now # 重置計時器以計算正確的 dt
            else:
                self.get_logger().info("⏳ Waiting for initial joint states from /joint_states...", throttle_duration_sec=1.0)
                # 在等待期間，我們仍然可以更新 PyBullet 的視覺化，讓我們看到手臂從初始位置移動到真實位置的過程
                if self.pybullet_ready and p.getConnectionInfo()['connectionMethod'] == p.GUI:
                    q_full = [0.0] * self.num_movable_joints
                    for pb_idx, (target_arm, control_idx) in self.pb_idx_to_control_map.items():
                        if pb_idx in self.movable_joints_map:
                            movable_idx = self.movable_joints_map[pb_idx]
                            q_full[movable_idx] = self.real_positions[target_arm][control_idx]
                    with self.pb_lock:
                        for i, pb_idx in enumerate(self.movable_joints):
                            p.resetJointState(self.robot_id, pb_idx, q_full[i])
                return # 跳過此控制迴圈

       # 穩定 dt，防止運算延遲導致步伐跳動
        raw_dt = (now - self.last_loop_time).nanoseconds / 1e9
        dt = max(0.01, min(raw_dt, 0.05))
        self.last_loop_time = now

        # 🔴 核心修正 2：每個迴圈只更新「一次」PyBullet 完整的物理狀態
        q_full = [0.0] * self.num_movable_joints
        filtered_cloud = None
        if self.pybullet_ready:
            # 準備完整關節陣列
            for pb_idx, (target_arm, control_idx) in self.pb_idx_to_control_map.items():
                if pb_idx in self.movable_joints_map:
                    movable_idx = self.movable_joints_map[pb_idx]
                    q_full[movable_idx] = self.real_positions[target_arm][control_idx]
            
            with self.pb_lock:
                # 統一更新一次關節
                for i, pb_idx in enumerate(self.movable_joints):
                    p.resetJointState(self.robot_id, pb_idx, q_full[i])

                # 同一個關節狀態快照只建立一次完整雙臂過濾幾何。
                self_filter_segments = self._get_bimanual_self_filter_segments()

            # 同一個點雲快照只做一次 self-filter，左右臂共用。
            with self.pc_lock:
                cloud_snapshot = (
                    self.point_cloud.copy() if self.point_cloud is not None else None
                )
            filtered_cloud = self._filter_robot_points(
                cloud_snapshot, *self_filter_segments
            )

            with self.pb_lock:
                # 視覺化也改用 self-filter 後的點雲。
                if p.getConnectionInfo()['connectionMethod'] == p.GUI:
                    if self.point_cloud_visual_id is not None:
                        p.removeUserDebugItem(self.point_cloud_visual_id)
                        self.point_cloud_visual_id = None
                    if filtered_cloud is not None and len(filtered_cloud) > 0:
                        self.point_cloud_visual_id = p.addUserDebugPoints(
                            pointPositions=filtered_cloud,
                            pointColorsRGB=[[1, 0, 0]] * len(filtered_cloud),
                            pointSize=3.0
                        )

        # 把 q_full 存起來，等一下傳給 CBF 用
        self.current_q_full = q_full

        for arm in self.arms:
            elapsed_vr = (now - self.last_msg_time[arm]).nanoseconds / 1e9
            if self.is_connected[arm] and elapsed_vr > self.timeout_sec:
                self.is_connected[arm] = False
                self.target_positions[arm] = list(self.start_pose)

            max_step = (self.max_vel_teleop if self.is_connected[arm] else self.max_vel_failsafe) * dt
            temp_full_pos = []

            dof = self.cbf_dof if hasattr(self, 'cbf_dof') else 7
            arm_target = np.array(self.target_positions[arm][:dof])
            arm_cmd = np.array(self.current_positions[arm][:dof])

            # 🔴 核心修正：先算出「最平滑的理想步伐 (dq_nominal)」
            ideal_smoothed_pos = self.alpha * arm_target + (1.0 - self.alpha) * arm_cmd if self.is_connected[arm] else arm_target
            ideal_step = ideal_smoothed_pos - arm_cmd
            ideal_step = np.clip(ideal_step, -max_step, max_step) # 限制最大速度
            
            dq_nominal = ideal_step / dt

            if self.use_cbf and self.pybullet_ready and self.is_connected[arm]:
                # 把完美的 dq_nominal 交給 CBF 把關
                dq_safe = (
                    self._compute_safe_velocity(
                        arm, dq_nominal, dt, q_full, filtered_cloud
                    )
                    if dt > 0 else np.zeros(dof)
                )
                
                # 直接積分安全速度，保證 100% 平滑！
                arm_next = arm_cmd + dq_safe * dt
                final_arm_pos = np.clip(arm_next, self.q_min[:dof], self.q_max[:dof])
                temp_full_pos.extend(final_arm_pos.tolist())
            else:
                # 無 CBF 時，直接套用平滑的 ideal_step
                arm_next = arm_cmd + ideal_step
                temp_full_pos.extend(arm_next.tolist())

            # 處理夾爪 (純 Teleop)
            num_total_joints = len(self.current_positions[arm])
            for i in range(dof, num_total_joints):
                target = self.target_positions[arm][i]
                current = self.current_positions[arm][i]
                smoothed = self.alpha * target + (1.0 - self.alpha) * current if self.is_connected[arm] else target
                diff = smoothed - current
                step = max(-max_step, min(diff, max_step))
                temp_full_pos.append(current + step)

            self.current_positions[arm] = temp_full_pos

            # 發布命令
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
