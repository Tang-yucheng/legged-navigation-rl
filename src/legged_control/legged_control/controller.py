
# 第三方库
import time
import torch
import numpy as np

from unitree_go.msg import LowCmd
from unitree_go.msg import LowState
from geometry_msgs.msg import Twist

from .common.crc import CRC
from .common.config import Config
from .common.remote_controller import RemoteController, KeyMap
from .common.command_helper import create_damping_cmd, create_zero_cmd, init_cmd
from .common.rotation_helper import get_gravity_orientation, transform_imu_data

from collections import deque


class ObsHistoryBuffer:
    def __init__(self, obs_dim=45, history_len=6):
        self.obs_dim = obs_dim
        self.history_len = history_len
        self.buffer = deque(maxlen=history_len)
        self.reset()

    def reset(self, init_obs=None):
        self.buffer.clear()
        if init_obs is not None:
            for _ in range(self.history_len):
                self.buffer.append(np.copy(init_obs).astype(np.float32))

    def append(self, obs):
        assert obs.shape[-1] == self.obs_dim, f"obs shape mismatch: {obs.shape}"
        self.buffer.append(np.copy(obs))

    def get(self):
        # 返回 shape: (1, obs_dim * history_len)
        # 左侧为最新的观测数据
        return np.concatenate(list(reversed(self.buffer)), axis=0).reshape(1, -1)
    
    def is_ready(self):
        return len(self.buffer) == self.history_len

class Controller:
    def __init__(self, config: Config) -> None:

        self.config = config
        self.remote_controller = RemoteController()

        # 观测队列
        self.obs_buffer = ObsHistoryBuffer(obs_dim=45, history_len=6)

        # Initialize the policy network
        self.policy = torch.jit.load(config.policy_path)
        # Initializing process variables
        self.low_cmd = LowCmd()
        self.low_state = LowState()
        self.cmd_vel = Twist()
        
        self.qj = np.zeros(config.num_actions, dtype=np.float32)
        self.dqj = np.zeros(config.num_actions, dtype=np.float32)
        self.action = np.zeros(config.num_actions, dtype=np.float32)
        self.target_dof_pos = config.default_angles_stand.copy()
        self.obs = np.zeros(config.num_obs, dtype=np.float32)
        self.cmd = np.array([0.0, 0.0, 0.0])
        
        # 初始化控制命令
        init_cmd(self.low_cmd)

        # 状态机初始化
        self.connect_state = False
        self.fsm_state = "IDLE"
    
    def step(self):
        if self.fsm_state == "IDLE":
            if self.connect_state:
                print("Successfully connected to the robot.", flush=True)
                print("Enter zero torque state.", flush=True)
                print("Waiting for the Button Start signal...", flush=True)
                self.zero_torque_state()
                self.fsm_state = "ZERO_TORQUE"
            else:
                self.wait_for_low_state()
        elif self.fsm_state == "ZERO_TORQUE":
            if self.remote_controller.button[KeyMap.start] == 1:
                print("Moving to default stand pos.", flush=True)
                self.count_move_default = 0
                self.state_move_default = False
                # move time 2s
                total_time = 2
                self.steps_move_default = int(total_time / self.config.control_dt)
                # record the current pos
                self.init_pos_move_default = np.zeros(len(self.config.leg_joint2motor_idx), dtype=np.float32)
                for i in range(len(self.config.leg_joint2motor_idx)):
                    self.init_pos_move_default[i] = self.low_state.motor_state[self.config.leg_joint2motor_idx[i]].q
                # 执行动作
                self.move_to_default_pos("stand")
                self.fsm_state = "MOVE_DEFAULT_STAND"
            else:
                self.zero_torque_state()
        elif self.fsm_state == "MOVE_DEFAULT_STAND":
            if self.state_move_default:
                print("Enter default pos state.", flush=True)
                print("Waiting for the Button A signal...", flush=True)
                self.default_pos_state()
                self.fsm_state = "DEFAULT_POS"
            else:
                self.move_to_default_pos("stand")
        elif self.fsm_state == "DEFAULT_POS":
            if self.remote_controller.button[KeyMap.A] == 1:
                print("Enter rl control mode.", flush=True)
                print("Press Button B to exit", flush=True)
                self.obs_buffer.reset()
                self.rl_control()
                self.fsm_state = "RL_CONTROL"
            else:
                self.default_pos_state()
        elif self.fsm_state == "RL_CONTROL":
            if self.remote_controller.button[KeyMap.B] == 1:
                print("Moving to default lie pos.", flush=True)
                self.count_move_default = 0
                self.state_move_default = False
                # move time 2s
                total_time = 2
                self.steps_move_default = int(total_time / self.config.control_dt)
                # record the current pos
                self.init_pos_move_default = np.zeros(len(self.config.leg_joint2motor_idx), dtype=np.float32)
                for i in range(len(self.config.leg_joint2motor_idx)):
                    self.init_pos_move_default[i] = self.low_state.motor_state[self.config.leg_joint2motor_idx[i]].q
                # 执行动作
                self.move_to_default_pos("lie")
                self.fsm_state = "MOVE_DEFAULT_LIE"
            else :
                self.rl_control()
                # print("rl", flush=True)
        elif self.fsm_state == "MOVE_DEFAULT_LIE":
            if self.state_move_default:
                print("Enter zero torque state.", flush=True)
                print("Waiting for the Button Start signal...", flush=True)
                self.zero_torque_state()
                self.fsm_state = "ZERO_TORQUE"
            else:
                self.move_to_default_pos("lie")

    def set_lowstate(self, msg: LowState):
        self.low_state = msg
        self.remote_controller.set(self.low_state.wireless_remote)

    def set_cmdvel(self, msg: Twist):
        self.cmd_vel = msg

    def get_lowcmd(self) -> LowCmd:
        self.low_cmd.crc = CRC().Crc(self.low_cmd)
        return self.low_cmd

    def wait_for_low_state(self):
        if self.low_state.tick != 0:
            self.connect_state = True

    def zero_torque_state(self):
        create_zero_cmd(self.low_cmd)

    def move_to_default_pos(self, posname):
        # move to default pos
        if self.count_move_default < self.steps_move_default:
            # 动作未完成，继续执行
            self.count_move_default += 1
            alpha = self.count_move_default / self.steps_move_default
            if posname == "stand":
                default_angles = self.config.default_angles_stand
            elif posname == "lie":
                default_angles = self.config.default_angles_lie
            for i in range(len(self.config.leg_joint2motor_idx)):
                motor_idx = self.config.leg_joint2motor_idx[i]
                target_pos = default_angles[i]
                self.low_cmd.motor_cmd[motor_idx].q = self.init_pos_move_default[i] * (1 - alpha) + target_pos * alpha
                self.low_cmd.motor_cmd[motor_idx].dq = 0.0
                self.low_cmd.motor_cmd[motor_idx].kp = self.config.kps[i]
                self.low_cmd.motor_cmd[motor_idx].kd = self.config.kds[i]
                self.low_cmd.motor_cmd[motor_idx].tau = 0.0
        else:
            # 动作已完成，标记完成
            self.state_move_default = True

    def default_pos_state(self):
        for i in range(len(self.config.leg_joint2motor_idx)):
            motor_idx = self.config.leg_joint2motor_idx[i]
            self.low_cmd.motor_cmd[motor_idx].q = self.config.default_angles_stand[i]
            self.low_cmd.motor_cmd[motor_idx].dq = 0.0
            self.low_cmd.motor_cmd[motor_idx].kp = self.config.kps[i]
            self.low_cmd.motor_cmd[motor_idx].kd = self.config.kds[i]
            self.low_cmd.motor_cmd[motor_idx].tau = 0.0

    def rl_control(self):
        # Get the current joint position and velocity
        for i in range(len(self.config.leg_joint2motor_idx)):
            self.qj[i] = self.low_state.motor_state[self.config.leg_joint2motor_idx[i]].q
            self.dqj[i] = self.low_state.motor_state[self.config.leg_joint2motor_idx[i]].dq

        # imu_state quaternion: w, x, y, z
        quat = self.low_state.imu_state.quaternion
        ang_vel = np.array([self.low_state.imu_state.gyroscope], dtype=np.float32)

        # create observation
        gravity_orientation = get_gravity_orientation(quat)
        qj_obs = self.qj.copy()
        dqj_obs = self.dqj.copy()
        qj_obs = (qj_obs - self.config.default_angles_stand) * self.config.dof_pos_scale
        dqj_obs = dqj_obs * self.config.dof_vel_scale
        ang_vel = ang_vel * self.config.ang_vel_scale

        # # 通过遥控器控制
        # self.cmd[0] = self.remote_controller.ly
        # self.cmd[1] = self.remote_controller.lx * -1
        # self.cmd[2] = self.remote_controller.rx * -1
        # self.cmd *= self.config.max_cmd
        # 通过cmd_vel控制
        self.cmd[0] = self.cmd_vel.linear.x
        self.cmd[1] = self.cmd_vel.linear.y
        self.cmd[2] = self.cmd_vel.angular.z

        num_actions = self.config.num_actions
        # self.obs[:3] = ang_vel
        # self.obs[3:6] = gravity_orientation
        # self.obs[6:9] = self.cmd * self.config.cmd_scale
        # self.obs[9 : 9 + num_actions] = qj_obs
        # self.obs[9 + num_actions : 9 + num_actions * 2] = dqj_obs
        # self.obs[9 + num_actions * 2 : 9 + num_actions * 3] = self.action
        # # Get the action from the policy network
        # obs_tensor = torch.from_numpy(self.obs).unsqueeze(0)
        # self.action = self.policy(obs_tensor).detach().numpy().squeeze()

        self.obs[:3] = self.cmd * self.config.cmd_scale
        print(self.cmd, flush=True)
        self.cmd[:] = 0

        self.obs[3:6] = ang_vel
        self.obs[6:9] = gravity_orientation
        self.obs[9 : 9 + num_actions] = qj_obs
        self.obs[9 + num_actions : 9 + num_actions * 2] = dqj_obs
        self.obs[9 + num_actions * 2 : 9 + num_actions * 3] = self.action
        # Get the action from the policy network
        self.obs_buffer.append(self.obs)        # 加入历史缓冲
        if self.obs_buffer.is_ready():
            obs_history = self.obs_buffer.get()     # shape: (1, 270)
            obs_tensor = torch.from_numpy(obs_history).float()
            self.action = self.policy(obs_tensor).detach().numpy().squeeze()
        else:
            print("[INFO] Buffer not ready, skipping control step.", flush=True)
        
        # transform action to target_dof_pos
        target_dof_pos = self.config.default_angles_stand + self.action * self.config.action_scale

        # Build low cmd
        for i in range(len(self.config.leg_joint2motor_idx)):
            motor_idx = self.config.leg_joint2motor_idx[i]
            self.low_cmd.motor_cmd[motor_idx].q = target_dof_pos[i]
            self.low_cmd.motor_cmd[motor_idx].dq = 0.0
            self.low_cmd.motor_cmd[motor_idx].kp = self.config.kps[i]
            self.low_cmd.motor_cmd[motor_idx].kd = self.config.kds[i]
            self.low_cmd.motor_cmd[motor_idx].tau = 0.0
