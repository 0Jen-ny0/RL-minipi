import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

from whole_body_tracking.assets import ASSET_DIR


NATURAL_FREQ = 10 * 2.0 * 3.1415926535  # 10Hz
DAMPING_RATIO = 2.0
MM2_TO_M2 = 1e-6

# (J_rotor + J_reducer)* 1e-6 * N^2
ARMATURE_3536 = (1.922 + 0.008)   * MM2_TO_M2 * (36**2)
ARMATURE_4438 = (9.12  + 0.03)    * MM2_TO_M2 * (36**2)
ARMATURE_5047 = (9.919 + 0.0426)  * MM2_TO_M2 * (36**2)
ARMATURE_6056 = (25.586+ 0.066)   * MM2_TO_M2 * (36**2)
ARMATURE_7256 = (41.038+ 0.341)   * MM2_TO_M2 * (36**2)

STIFFNESS_3536 = ARMATURE_3536 * NATURAL_FREQ**2
STIFFNESS_4438 = ARMATURE_4438 * NATURAL_FREQ**2
STIFFNESS_5047 = ARMATURE_5047 * NATURAL_FREQ**2
STIFFNESS_6056 = ARMATURE_6056 * NATURAL_FREQ**2
STIFFNESS_7256 = ARMATURE_7256 * NATURAL_FREQ**2

DAMPING_3536 = 2.0 * DAMPING_RATIO * ARMATURE_3536 * NATURAL_FREQ
DAMPING_4438 = 2.0 * DAMPING_RATIO * ARMATURE_4438 * NATURAL_FREQ
DAMPING_5047 = 2.0 * DAMPING_RATIO * ARMATURE_5047 * NATURAL_FREQ
DAMPING_6056 = 2.0 * DAMPING_RATIO * ARMATURE_6056 * NATURAL_FREQ
DAMPING_7256 = 2.0 * DAMPING_RATIO * ARMATURE_7256 * NATURAL_FREQ


MINIPI_CFG = ArticulationCfg(
    spawn=sim_utils.UrdfFileCfg(
        fix_base=False,
        replace_cylinders_with_capsules=True,
        asset_path=f"{ASSET_DIR}/hightorque_minipi/urdf/hightorque_minipi.urdf",
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True, solver_position_iteration_count=8, solver_velocity_iteration_count=4
        ),
        joint_drive=sim_utils.UrdfConverterCfg.JointDriveCfg(
            gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=0, damping=0)
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.45),
        joint_pos={
            ".*_hip_pitch_joint": -0.30,
            ".*_hip_roll_joint": 0.0,
            ".*_thigh_joint": 0.0,
            ".*_calf_joint": 0.65,
            ".*_ankle_pitch_joint": -0.35,
            ".*_ankle_roll_joint": 0.0,
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_hip_pitch_joint",
                ".*_hip_roll_joint",
                ".*_thigh_joint",
                ".*_calf_joint",
            ],
            effort_limit_sim=20.0,
            velocity_limit_sim=5.0,
            stiffness={
                ".*_hip_pitch_joint": STIFFNESS_5047,
                ".*_hip_roll_joint": STIFFNESS_5047,
                ".*_thigh_joint": STIFFNESS_5047,
                ".*_calf_joint": STIFFNESS_5047,
            },
            damping={
                ".*_hip_pitch_joint": DAMPING_5047,
                ".*_hip_roll_joint": DAMPING_5047,
                ".*_thigh_joint": DAMPING_5047,
                ".*_calf_joint": DAMPING_5047,
            },
            armature={
                ".*_hip_pitch_joint": ARMATURE_5047,
                ".*_hip_roll_joint": ARMATURE_5047,
                ".*_thigh_joint": ARMATURE_5047,
                ".*_calf_joint": ARMATURE_5047,
            },
        ),
        "feet": ImplicitActuatorCfg(
            effort_limit_sim=20.0,
            velocity_limit_sim=5.0,
            joint_names_expr=[".*_ankle_pitch_joint", ".*_ankle_roll_joint"],
            stiffness=2.0 * STIFFNESS_5047,
            damping=2.0 * DAMPING_5047,
            armature=2.0 * ARMATURE_5047,
        ),
    },
)

MINIPI_ACTION_SCALE = {}
for a in MINIPI_CFG.actuators.values():
    e = a.effort_limit_sim
    s = a.stiffness
    names = a.joint_names_expr
    if not isinstance(e, dict):
        e = {n: e for n in names}
    if not isinstance(s, dict):
        s = {n: s for n in names}
    for n in names:
        if n in e and n in s and s[n]:
            MINIPI_ACTION_SCALE[n] = 0.25 * e[n] / s[n]
