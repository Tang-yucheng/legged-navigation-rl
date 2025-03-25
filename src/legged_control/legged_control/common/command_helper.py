from unitree_go.msg import LowCmd
from unitree_go.msg import LowState

def create_damping_cmd(cmd: LowCmd):
    size = len(cmd.motor_cmd)
    for i in range(size):
        cmd.motor_cmd[i].q = 0.0
        cmd.motor_cmd[i].dq = 0.0
        cmd.motor_cmd[i].kp = 0.0
        cmd.motor_cmd[i].kd = 8.0
        cmd.motor_cmd[i].tau = 0.0


def create_zero_cmd(cmd: LowCmd):
    size = len(cmd.motor_cmd)
    for i in range(size):
        cmd.motor_cmd[i].q = 0.0
        cmd.motor_cmd[i].dq = 0.0
        cmd.motor_cmd[i].kp = 0.0
        cmd.motor_cmd[i].kd = 0.0
        cmd.motor_cmd[i].tau = 0.0

def init_cmd(cmd: LowCmd):
    cmd.head[0] = 0xFE
    cmd.head[1] = 0xEF
    cmd.gpio = 0
    PosStopF = 2.146e9
    VelStopF = 16000.0
    size = len(cmd.motor_cmd)
    for i in range(size):
        cmd.motor_cmd[i].mode = 1
        cmd.motor_cmd[i].q = PosStopF
        cmd.motor_cmd[i].dq = VelStopF
        cmd.motor_cmd[i].kp = 0.0
        cmd.motor_cmd[i].kd = 0.0
        cmd.motor_cmd[i].tau = 0.0