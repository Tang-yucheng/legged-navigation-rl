"""四足强化学习控制节点（无外部感知）
"""

# ros库
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

# msg
from unitree_go.msg import LowCmd
from unitree_go.msg import LowState

# 第三方库
import time
import torch
import numpy as np
from importlib.resources import path

from legged_control import policy

from .common.config import Config
from .controller import Controller

class Legged_Control(Node):
    def __init__(self):
        super().__init__('legged_control_node')

        # 参数声明
        self.init_parameter()

        # 创建发布者、订阅者、服务端对象
        self.pub_lowcmd = self.create_publisher(LowCmd, '/lowcmd', 5)
        self.sub_lowstate = self.create_subscription(LowState, '/lowstate', self.lowstate_callback, qos_profile_sensor_data)

        # 加载控制器
        self.controller = Controller(self.config)

        # 创建定时器对象
        self.tim_control = self.create_timer(self.config.control_dt, self.control_callback)

        self.lasttime = self.get_clock().now()

    def lowstate_callback(self, msg):
        self.controller.set_lowstate(msg)

    def control_callback(self):
        # 计算实际频率
        self.time = self.get_clock().now()
        freq = 1 / ((self.time - self.lasttime).nanoseconds / 1e9)
        # self.get_logger().info(f"frequency: {freq}")
        self.lasttime = self.time

        # 计算控制
        self.controller.step()

        # 命令发送
        msg_lowcmd = self.controller.get_lowcmd()
        self.pub_lowcmd.publish(msg_lowcmd)

    def init_parameter(self):
        # 读取参数
        self.declare_parameter("control_frequency", 50.0)
        control_frequency = self.get_parameter("control_frequency").value
        control_dt = 1 / control_frequency

        # self.policy_path = self.declare_parameter("policy_path").get_parameter_value().replace("{LEGGED_GYM_ROOT_DIR}", LEGGED_GYM_ROOT_DIR)
        self.declare_parameter("policy_path", "")
        policy_path = self.get_parameter("policy_path").value
        with path(policy, policy_path) as get_path:
            policy_path = get_path

        self.declare_parameter("leg_joint2motor_idx", [0])
        leg_joint2motor_idx = self.get_parameter("leg_joint2motor_idx").value
        self.declare_parameter("kps", [0.0])
        kps = self.get_parameter("kps").value
        self.declare_parameter("kds", [0.0])
        kds = self.get_parameter("kds").value
        self.declare_parameter("default_angles_stand", [0.0])
        default_angles_stand = self.get_parameter("default_angles_stand").value
        self.declare_parameter("default_angles_lie", [0.0])
        default_angles_lie = self.get_parameter("default_angles_lie").value

        self.declare_parameter("num_actions", 12)
        num_actions = self.get_parameter("num_actions").value
        self.declare_parameter("num_obs", 48)
        num_obs = self.get_parameter("num_obs").value

        self.declare_parameter("ang_vel_scale", 0.25)
        ang_vel_scale = self.get_parameter("ang_vel_scale").value
        self.declare_parameter("dof_pos_scale", 0.25)
        dof_pos_scale = self.get_parameter("dof_pos_scale").value
        self.declare_parameter("dof_vel_scale", 0.25)
        dof_vel_scale = self.get_parameter("dof_vel_scale").value
        self.declare_parameter("action_scale", 0.25)
        action_scale = self.get_parameter("action_scale").value
        
        self.declare_parameter("cmd_scale", [0.0])
        cmd_scale = self.get_parameter("cmd_scale").value
        self.declare_parameter("max_cmd", [0.0])
        max_cmd = self.get_parameter("max_cmd").value

        # 参数对象
        self.config = Config(
            control_frequency,
            control_dt,
            policy_path,
            leg_joint2motor_idx,
            kps,
            kds,
            default_angles_stand,
            default_angles_lie,
            num_actions,
            num_obs,
            ang_vel_scale,
            dof_pos_scale,
            dof_vel_scale,
            action_scale,
            cmd_scale,
            max_cmd
        )

def main(args=None):
    rclpy.init(args=args)
    node = Legged_Control()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
