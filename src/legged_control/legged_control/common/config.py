
from dataclasses import dataclass

@dataclass
class Config:
    control_frequency: float
    control_dt: float
    
    policy_path: str

    leg_joint2motor_idx: list[int]
    kps: list[float]
    kds: list[float]
    default_angles_stand: list[float]
    default_angles_lie: list[float]

    num_actions: int
    num_obs: int

    ang_vel_scale: float
    dof_pos_scale: float
    dof_vel_scale: float
    action_scale: float

    cmd_scale: list[float]
    max_cmd: list[float]
