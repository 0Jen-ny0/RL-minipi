from isaaclab.utils import configclass
from whole_body_tracking.robots.minipi import MINIPI_ACTION_SCALE, MINIPI_CFG
from whole_body_tracking.tasks.tracking.config.minipi.agents.rsl_rl_ppo_cfg import LOW_FREQ_SCALE
from whole_body_tracking.tasks.tracking.tracking_env_cfg import TrackingEnvCfg


@configclass
class MiniPiFlatEnvCfg(TrackingEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        self.scene.robot = MINIPI_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        self.actions.joint_pos.scale = MINIPI_ACTION_SCALE
        self.commands.motion.anchor_body_name = "base_link"
        self.commands.motion.body_names = [
            "base_link",

            "l_hip_pitch_link",
            "l_hip_roll_link",
            "l_thigh_link",
            "l_calf_link",
            "l_ankle_roll_link",

            "r_hip_pitch_link",
            "r_hip_roll_link",
            "r_thigh_link",
            "r_calf_link",
            "r_ankle_roll_link",
        ]


@configclass
class MiniPiFlatWoStateEstimationEnvCfg(MiniPiFlatEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.observations.policy.motion_anchor_pos_b = None
        self.observations.policy.base_lin_vel = None


@configclass
class MiniPiFlatLowFreqEnvCfg(MiniPiFlatEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.decimation = round(self.decimation / LOW_FREQ_SCALE)
        self.rewards.action_rate_l2.weight *= LOW_FREQ_SCALE
