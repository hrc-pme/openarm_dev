#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray


class UnityAdapterNode(Node):
    ARM_JOINT_COUNT = 7
    COMMAND_COUNT = 8
    CONTROL_PERIOD_SEC = 0.02
    GRIPPER_MAX_POSITION = 0.044
    READY_RETREAT_J4 = 2.0
    READY_FINAL_J4 = math.radians(80.0)

    def __init__(self):
        super().__init__('unity_adapter_node')

        self.arms = ['left', 'right']

        self.declare_parameter('vr_timeout_sec', 2.0)
        self.declare_parameter('teleop_signal_timeout_sec', 0.5)
        self.declare_parameter('max_arm_velocity', 1.20)
        self.declare_parameter('max_arm_acceleration', 0.70)
        self.declare_parameter('max_gripper_velocity', 0.10)
        self.declare_parameter('max_gripper_acceleration', 0.50)
        self.declare_parameter('smoothing_alpha', 0.3)
        # Unity 原本發布的夾爪關節值已是實機位置單位（m），
        # 預設直接映射，不再重複乘上 0.044。
        self.declare_parameter('gripper_input_normalized', False)
        self.declare_parameter('startup_max_arm_velocity', 0.20)
        self.declare_parameter('startup_max_arm_acceleration', 0.40)
        # 實機在 J1/J4/J7 可有約 0.06–0.085 rad 的穩態追蹤誤差。
        self.declare_parameter('startup_stage_tolerance', 0.10)
        # 可單獨調整第一階段的 J4 容差；預設與其他 Ready 關節相同。
        self.declare_parameter('startup_retreat_j4_tolerance', 0.10)
        self.declare_parameter('startup_stage_timeout_sec', 30.0)
        self.declare_parameter('startup_settle_time_sec', 0.5)
        self.declare_parameter('joint_state_timeout_sec', 1.0)

        # 關節硬限位 (J1~J7 + J8夾爪)
        self.joint_limits = {
            'j1': (math.radians(-80), math.radians(200)),
            'j2': (math.radians(-100), math.radians(100)),
            'j3': (math.radians(-90), math.radians(90)),
            'j4': (math.radians(0), math.radians(140)),
            'j5': (math.radians(-90), math.radians(90)),
            'j6': (math.radians(-45), math.radians(45)),
            'j7': (math.radians(-90), math.radians(90)),
            'j8': (0.0, self.GRIPPER_MAX_POSITION),
        }

        # 狀態管理
        self.is_connected = {'left': False, 'right': False}
        self.last_msg_time = {arm: self.get_clock().now() for arm in self.arms}
        self.timeout_sec = max(
            0.1, float(self.get_parameter('vr_timeout_sec').value))
        self.last_joint_state_time = {arm: None for arm in self.arms}
        self.vr_blocked_warning_sent = {arm: False for arm in self.arms}
        # 必須先收到 Unity 持續發出的按住訊號，才允許自動或 VR 動作。
        self.teleop_enabled = False
        self.last_teleop_signal_time = None
        self.teleop_signal_timeout_sec = max(
            0.1, float(self.get_parameter('teleop_signal_timeout_sec').value))

        # 在收到完整 /joint_states 前，不會發布任何控制命令。
        self.is_initialized = {arm: False for arm in self.arms}
        self.actual_positions = {arm: [0.0] * self.COMMAND_COUNT for arm in self.arms}
        self.current_positions = {arm: [0.0] * self.COMMAND_COUNT for arm in self.arms}
        self.target_positions = {arm: [0.0] * self.COMMAND_COUNT for arm in self.arms}
        self.command_velocities = {arm: [0.0] * self.COMMAND_COUNT for arm in self.arms}


        # 保守的遙操作速度與加速度限制（手臂單位 rad，夾爪單位 m）。
        self.max_arm_velocity = max(
            0.01, float(self.get_parameter('max_arm_velocity').value))
        self.max_arm_acceleration = max(
            0.01, float(self.get_parameter('max_arm_acceleration').value))
        self.max_gripper_velocity = max(
            0.001, float(self.get_parameter('max_gripper_velocity').value))
        self.max_gripper_acceleration = max(
            0.001, float(self.get_parameter('max_gripper_acceleration').value))
        self.alpha = max(
            0.0, min(float(self.get_parameter('smoothing_alpha').value), 1.0))
        self.gripper_input_normalized = bool(
            self.get_parameter('gripper_input_normalized').value)
        self.startup_max_arm_velocity = max(
            0.01, float(self.get_parameter('startup_max_arm_velocity').value))
        self.startup_max_arm_acceleration = max(
            0.01, float(self.get_parameter('startup_max_arm_acceleration').value))
        self.startup_stage_tolerance = max(
            0.005, float(self.get_parameter('startup_stage_tolerance').value))
        self.startup_retreat_j4_tolerance = max(
            self.startup_stage_tolerance,
            float(self.get_parameter('startup_retreat_j4_tolerance').value))
        self.startup_stage_timeout_sec = max(
            1.0, float(self.get_parameter('startup_stage_timeout_sec').value))
        self.startup_settle_time_sec = max(
            0.0, float(self.get_parameter('startup_settle_time_sec').value))
        self.joint_state_timeout_sec = max(
            0.1, float(self.get_parameter('joint_state_timeout_sec').value))

        self.ready_for_vr = False
        self.ready_sequence_active = False
        self.ready_sequence_started = False
        self.ready_stage_index = -1
        self.ready_stage_started = None
        self.ready_stage_reached_since = None
        self.ready_waypoints = {}
        self.ready_stage_names = ('後抬 J4', 'hands_up', '桌面 Ready pose')


        self.joint_names = {
            arm: [f'openarm_{arm}_joint{i}' for i in range(1, 8)]
            + [f'openarm_{arm}_finger_joint1']
            for arm in self.arms
        }

        self.subs = {}
        self.pubs = {}
        self.gripper_pubs = {}

        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        self.teleop_enable_sub = self.create_subscription(
            Bool, '/unity_adapter/teleop_enable',
            self.teleop_enable_callback, 10)

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
        self.control_timer = self.create_timer(self.CONTROL_PERIOD_SEC, self.control_loop)

        self.get_logger().info(
            'Unity Adapter 已啟動；按住左手拇指與中指後才會進入 Ready pose。')
        self.get_logger().info(
            f'Ready 容差：一般={self.startup_stage_tolerance:.3f} rad，'
            f'後抬 J4={self.startup_retreat_j4_tolerance:.3f} rad。')

    def _start_ready_sequence(self, now):
        if not self.teleop_enabled:
            return False
        if self.ready_sequence_started or self.ready_sequence_active:
            return False
        if not all(self.is_initialized.values()):
            self.get_logger().error(
                '自動 Ready 已取消：尚未收到完整的雙臂 joint state。')
            return False
        if not self._joint_states_are_fresh(now):
            self.get_logger().error(
                '自動 Ready 已取消：雙臂 joint state 已逾時。')
            return False

        self.ready_sequence_started = True
        self.ready_waypoints = {}
        for arm in self.arms:
            measured = list(self.actual_positions[arm])

            retreat = list(measured)
            retreat[3] = self.READY_RETREAT_J4
            retreat[7] = 0.0

            hands_up = [0.0] * self.COMMAND_COUNT
            hands_up[3] = self.READY_RETREAT_J4

            ready = list(hands_up)
            ready[3] = self.READY_FINAL_J4

            self.ready_waypoints[arm] = (retreat, hands_up, ready)
            self.is_connected[arm] = False
            self.current_positions[arm] = list(measured)
            self.target_positions[arm] = list(retreat)
            self.command_velocities[arm] = [0.0] * self.COMMAND_COUNT
            self.vr_blocked_warning_sent[arm] = False

        self.ready_for_vr = False
        self.ready_sequence_active = True
        self.ready_stage_index = 0
        self.ready_stage_started = now
        self.ready_stage_reached_since = None

        self.get_logger().warning(
            '自動 Ready sequence 已開始：後抬 J4 → hands_up → '
            '80 度桌面 Ready pose。')
        return True

    def _joint_states_are_fresh(self, now):
        for arm in self.arms:
            last_time = self.last_joint_state_time[arm]
            if last_time is None:
                return False
            age = (now - last_time).nanoseconds / 1e9
            if age > self.joint_state_timeout_sec:
                return False
        return True

    def _abort_ready_sequence(self, reason: str):
        self.ready_sequence_active = False
        self.ready_for_vr = False
        self.ready_stage_index = -1
        self.ready_stage_started = None
        self.ready_stage_reached_since = None
        for arm in self.arms:
            self.is_connected[arm] = False
            self._hold_actual_position(arm)
        self.get_logger().error(
            f'Ready sequence 已停止並保持實際姿態：{reason}')

    def _ready_joint_tolerance(self, joint_index):
        if self.ready_stage_index == 0 and joint_index == 3:
            return self.startup_retreat_j4_tolerance
        return self.startup_stage_tolerance

    def _ready_stage_error_summary(self):
        """回報目前階段所有未達標的關節與各自容差。"""
        if (
            self.ready_stage_index < 0
            or self.ready_stage_index >= len(self.ready_stage_names)
        ):
            return ''

        summaries = []
        for arm in self.arms:
            waypoint = self.ready_waypoints[arm][self.ready_stage_index]
            for joint_index in range(self.ARM_JOINT_COUNT):
                actual = self.actual_positions[arm][joint_index]
                target = waypoint[joint_index]
                error = actual - target
                tolerance = self._ready_joint_tolerance(joint_index)
                if abs(error) > tolerance:
                    summaries.append(
                        f'{arm} J{joint_index + 1}: actual={actual:.3f}, '
                        f'target={target:.3f}, error={error:+.3f}, '
                        f'tolerance={tolerance:.3f} rad')
        return '; '.join(summaries) or (
            '所有關節目前在容差內，但未持續達標 '
            f'{self.startup_settle_time_sec:.2f} 秒')

    def _update_ready_sequence(self, now):
        if not self.ready_sequence_active or not self.teleop_enabled:
            return

        stage_elapsed = (
            now - self.ready_stage_started).nanoseconds / 1e9
        if stage_elapsed > self.startup_stage_timeout_sec:
            stage_name = self.ready_stage_names[self.ready_stage_index]
            error_summary = self._ready_stage_error_summary()
            reason = f'{stage_name} 階段逾時'
            if error_summary:
                reason += f'（{error_summary}）'
            self._abort_ready_sequence(reason)
            return

        reached = all(
            abs(
                self.actual_positions[arm][joint_index]
                - self.ready_waypoints[arm][self.ready_stage_index][joint_index]
            ) <= self._ready_joint_tolerance(joint_index)
            for arm in self.arms
            for joint_index in range(self.ARM_JOINT_COUNT)
        )
        if not reached:
            self.ready_stage_reached_since = None
            return

        if self.ready_stage_reached_since is None:
            self.ready_stage_reached_since = now
            return

        settled_for = (
            now - self.ready_stage_reached_since).nanoseconds / 1e9
        if settled_for < self.startup_settle_time_sec:
            return

        self.ready_stage_index += 1
        self.ready_stage_reached_since = None

        if self.ready_stage_index >= len(self.ready_stage_names):
            self.ready_sequence_active = False
            self.ready_for_vr = True
            self.ready_stage_index = -1
            self.ready_stage_started = None
            for arm in self.arms:
                self.is_connected[arm] = False
                self._hold_actual_position(arm)
                self.target_positions[arm][7] = 0.0
                self.vr_blocked_warning_sent[arm] = False
            self.get_logger().info(
                'Ready pose 已到達，現在開始接受 Unity VR 指令。')
            return

        self.ready_stage_started = now
        for arm in self.arms:
            self.target_positions[arm] = list(
                self.ready_waypoints[arm][self.ready_stage_index])
            self.command_velocities[arm] = [0.0] * self.COMMAND_COUNT

        self.get_logger().info(
            f'Ready sequence 進入階段：'
            f'{self.ready_stage_names[self.ready_stage_index]}')

    def joint_state_callback(self, msg: JointState):
        if len(msg.position) < len(msg.name):
            self.get_logger().warning(
                '忽略格式錯誤的 /joint_states：position 數量少於 name。')
            return

        position_by_name = {
            name: float(msg.position[i])
            for i, name in enumerate(msg.name)
            if math.isfinite(msg.position[i])
        }
        now = self.get_clock().now()

        for arm in self.arms:
            names = self.joint_names[arm]
            if not all(name in position_by_name for name in names):
                continue

            measured = [position_by_name[name] for name in names]
            self.actual_positions[arm] = measured
            self.last_joint_state_time[arm] = now

            if not self.is_initialized[arm]:
                self.is_initialized[arm] = True
                self.current_positions[arm] = list(measured)
                self.target_positions[arm] = list(measured)
                self.command_velocities[arm] = [0.0] * self.COMMAND_COUNT
                self.get_logger().info(
                    f'{arm} arm 已由實際 joint state 初始化，將保持目前姿態。')

            # actual_positions 只用於回授與安全診斷。
            # Ready 逾時或 VR 中斷時，target_positions 必須鎖在
            # _hold_actual_position() 當下擷取的姿態，不能跟著重力下沉。

        if (
            self.teleop_enabled
            and not self.ready_sequence_started
            and all(self.is_initialized.values())
            and self._joint_states_are_fresh(now)
        ):
            self._start_ready_sequence(now)

    def teleop_enable_callback(self, msg: Bool):
        now = self.get_clock().now()
        self.last_teleop_signal_time = now
        enabled = bool(msg.data)
        self._set_teleop_enabled(enabled, now)

    def _set_teleop_enabled(self, enabled: bool, now):
        if enabled == self.teleop_enabled:
            return

        self.teleop_enabled = enabled
        for arm in self.arms:
            self.is_connected[arm] = False
            self.vr_blocked_warning_sent[arm] = False

        if not enabled:
            for arm in self.arms:
                if self.is_initialized[arm]:
                    self._hold_commanded_position(arm)
            self.get_logger().warning(
                '左手捏合已放開；雙臂保持最後送出的關節命令。')
            return

        if self.ready_sequence_active:
            # 暫停期間不累計 Ready 階段逾時，重新按住後繼續原階段。
            self.ready_stage_started = now
            self.ready_stage_reached_since = None
            for arm in self.arms:
                self.target_positions[arm] = list(
                    self.ready_waypoints[arm][self.ready_stage_index])
                self.command_velocities[arm] = [0.0] * self.COMMAND_COUNT
            self.get_logger().info('左手捏合已恢復；繼續 Ready sequence。')
        elif not self.ready_sequence_started:
            if all(self.is_initialized.values()) and self._joint_states_are_fresh(now):
                self._start_ready_sequence(now)
        elif self.ready_for_vr:
            self.get_logger().info(
                'Unity 已重新啟用 VR 控制；將恢復絕對關節角度對應。')
        else:
            self.get_logger().warning(
                'VR 控制已啟用，但 Ready sequence 曾因錯誤中止；'
                '請檢查現場後重新啟動 adapter。')

    @staticmethod
    def _command_index(name: str):
        name_lower = name.lower()
        if 'finger' in name_lower or 'gripper' in name_lower:
            return 7

        for idx in range(1, 8):
            if (
                f'joint{idx}' in name_lower
                or f'j{idx}' in name_lower
                or f'link{idx}' in name_lower
            ):
                return idx - 1
        return None

    def vr_command_callback(self, msg: JointState, arm: str):
        if not self.is_initialized[arm]:
            self.get_logger().warning(
                f'忽略 {arm} VR 命令：尚未收到完整的實際 joint state。')
            return
        if not self.teleop_enabled:
            if not self.vr_blocked_warning_sent[arm]:
                self.get_logger().warning(
                    f'忽略 {arm} VR 命令：Unity 已停用 VR 控制。')
                self.vr_blocked_warning_sent[arm] = True
            return
        if self.ready_sequence_active or not self.ready_for_vr:
            if not self.vr_blocked_warning_sent[arm]:
                self.get_logger().warning(
                    f'忽略 {arm} VR 命令：自動 Ready sequence 尚未完成。')
                self.vr_blocked_warning_sent[arm] = True
            return

        self.vr_blocked_warning_sent[arm] = False


        if len(msg.position) < len(msg.name):
            self.get_logger().warning(
                f'忽略格式錯誤的 {arm} VR 命令：position 數量少於 name。')
            return

        new_target = list(self.target_positions[arm])
        parsed_commands = {}

        for i, name in enumerate(msg.name):
            target_idx = self._command_index(name)
            raw_position = float(msg.position[i])
            if target_idx is None or not math.isfinite(raw_position):
                continue

            if target_idx == 7 and self.gripper_input_normalized:
                raw_position *= self.GRIPPER_MAX_POSITION

            parsed_commands[target_idx] = raw_position

        if not parsed_commands:
            return

        reconnecting = not self.is_connected[arm]

        for target_idx, raw_position in parsed_commands.items():
            min_lim, max_lim = self.joint_limits[f'j{target_idx + 1}']
            new_target[target_idx] = max(
                min_lim, min(raw_position, max_lim))

        self.target_positions[arm] = new_target
        self.last_msg_time[arm] = self.get_clock().now()
        self.is_connected[arm] = True

        if reconnecting:
            self.get_logger().info(
                f'{arm} VR 已連線；使用 Unity 絕對關節角度。')

    @staticmethod
    def _limited_step(
        current,
        target,
        current_velocity,
        max_velocity,
        max_acceleration,
        dt,
    ):
        error = target - current
        if abs(error) < 1e-6:
            return target, 0.0

        # 若目前速度方向與目標相反，先停止，避免短暫往錯誤方向移動。
        if current_velocity * error < 0.0:
            current_velocity = 0.0

        desired_velocity = max(
            -max_velocity, min(error / dt, max_velocity))
        max_velocity_change = max_acceleration * dt
        velocity = current_velocity + max(
            -max_velocity_change,
            min(desired_velocity - current_velocity, max_velocity_change),
        )

        step = velocity * dt
        if abs(step) >= abs(error):
            return target, 0.0
        return current + step, velocity

    def _hold_actual_position(self, arm: str):
        measured = list(self.actual_positions[arm])
        self.current_positions[arm] = measured
        self.target_positions[arm] = list(measured)
        self.command_velocities[arm] = [0.0] * self.COMMAND_COUNT

    def _hold_commanded_position(self, arm: str):
        # 放開手勢時保持上一筆已發布的目標，避免量測追蹤誤差
        # 將命令位置瞬間改到較低的實際位置。
        self.target_positions[arm] = list(self.current_positions[arm])
        self.command_velocities[arm] = [0.0] * self.COMMAND_COUNT

    def control_loop(self):
        now = self.get_clock().now()
        if self.teleop_enabled and (
            self.last_teleop_signal_time is None or
            (now - self.last_teleop_signal_time).nanoseconds / 1e9
            > self.teleop_signal_timeout_sec
        ):
            self._set_teleop_enabled(False, now)
        dt = (now - self.last_loop_time).nanoseconds / 1e9
        self.last_loop_time = now
        if dt <= 0.0:
            return
        dt = min(dt, 0.05)
        if self.ready_sequence_active and not self._joint_states_are_fresh(now):
            self._abort_ready_sequence('joint state 回授逾時')



        for arm in self.arms:
            if not self.is_initialized[arm]:
                continue

            elapsed_vr = (now - self.last_msg_time[arm]).nanoseconds / 1e9
            if self.is_connected[arm] and elapsed_vr > self.timeout_sec:
                self.is_connected[arm] = False
                self._hold_actual_position(arm)
                self.get_logger().warning(
                    f'{arm} VR 命令逾時；保持實際姿態，不返回 start pose。')

            next_positions = []
            next_velocities = []
            for i in range(self.COMMAND_COUNT):
                current = self.current_positions[arm][i]
                target = self.target_positions[arm][i]
                if self.is_connected[arm]:
                    target = self.alpha * target + (1.0 - self.alpha) * current

                if i < self.ARM_JOINT_COUNT:
                    if self.ready_sequence_active:
                        max_velocity = self.startup_max_arm_velocity
                        max_acceleration = self.startup_max_arm_acceleration
                    else:
                        max_velocity = self.max_arm_velocity
                        max_acceleration = self.max_arm_acceleration
                else:
                    max_velocity = self.max_gripper_velocity
                    max_acceleration = self.max_gripper_acceleration

                position, velocity = self._limited_step(
                    current,
                    target,
                    self.command_velocities[arm][i],
                    max_velocity,
                    max_acceleration,
                    dt,
                )
                next_positions.append(position)
                next_velocities.append(velocity)

            self.current_positions[arm] = next_positions
            self.command_velocities[arm] = next_velocities

            self.pubs[arm].publish(
                Float64MultiArray(data=next_positions[:self.ARM_JOINT_COUNT]))
            self.gripper_pubs[arm].publish(
                Float64MultiArray(data=[next_positions[7]]))

        self._update_ready_sequence(now)


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
