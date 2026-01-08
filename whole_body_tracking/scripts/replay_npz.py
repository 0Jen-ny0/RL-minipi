""""
This script demonstrates how to replay a motion from a wandb registry artifact (.npz).

.. code-block:: bash

    python replay_motion.py --registry_name <entity>/<project>/<artifact>:latest
"""

import argparse
import numpy as np
import torch

from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Replay converted motions.")
parser.add_argument("--registry_name", type=str, required=True, help="The name of the wandb registry artifact.")
parser.add_argument("--dt", type=float, default=0.02, help="Simulation dt (seconds). Default 0.02 (=50Hz).")

# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# --- rest everything follows ---
import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation, ArticulationCfg, AssetBaseCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sim import SimulationContext
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

# Use MiniPi robot cfg
from whole_body_tracking.robots.minipi import MINIPI_CFG


@configclass
class ReplayMotionsSceneCfg(InteractiveSceneCfg):
    """Configuration for a replay motions scene."""

    ground = AssetBaseCfg(prim_path="/World/defaultGroundPlane", spawn=sim_utils.GroundPlaneCfg())

    sky_light = AssetBaseCfg(
        prim_path="/World/skyLight",
        spawn=sim_utils.DomeLightCfg(
            intensity=750.0,
            texture_file=f"{ISAAC_NUCLEUS_DIR}/Materials/Textures/Skies/PolyHaven/kloofendal_43d_clear_puresky_4k.hdr",
        ),
    )

    robot: ArticulationCfg = MINIPI_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")


def _load_motion_npz_from_wandb(registry_name: str) -> dict:
    if ":" not in registry_name:
        registry_name += ":latest"

    import pathlib
    import wandb

    api = wandb.Api()
    artifact = api.artifact(registry_name)
    motion_file = str(pathlib.Path(artifact.download()) / "motion.npz")

    data = np.load(motion_file, allow_pickle=True)
    # Convert to plain dict of arrays
    return {k: data[k] for k in data.files}


def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene):
    robot: Articulation = scene["robot"]
    sim_dt = sim.get_physics_dt()

    motion = _load_motion_npz_from_wandb(args_cli.registry_name)

    # Expected keys from csv_to_npz.py logger
    joint_pos = torch.from_numpy(motion["joint_pos"]).to(dtype=torch.float32, device=sim.device)  # (T, dof)
    joint_vel = torch.from_numpy(motion["joint_vel"]).to(dtype=torch.float32, device=sim.device)  # (T, dof)

    body_pos_w = torch.from_numpy(motion["body_pos_w"]).to(dtype=torch.float32, device=sim.device)  # (T, B, 3)
    body_quat_w = torch.from_numpy(motion["body_quat_w"]).to(dtype=torch.float32, device=sim.device)  # (T, B, 4)
    body_lin_vel_w = torch.from_numpy(motion["body_lin_vel_w"]).to(dtype=torch.float32, device=sim.device)  # (T, B, 3)
    body_ang_vel_w = torch.from_numpy(motion["body_ang_vel_w"]).to(dtype=torch.float32, device=sim.device)  # (T, B, 3)

    T = joint_pos.shape[0]
    time_step = torch.zeros(scene.num_envs, dtype=torch.long, device=sim.device)

    # Sanity: ensure dof matches robot
    if joint_pos.shape[1] != robot.num_joints:
        print(f"[WARN] Motion DoF={joint_pos.shape[1]} but robot DoF={robot.num_joints}. "
              f"If this is MiniPi, motion should be 12 DoF.")

    while simulation_app.is_running():
        time_step += 1
        reset_ids = time_step >= T
        time_step[reset_ids] = 0

        # Root state from body 0 (usually base)
        root_states = robot.data.default_root_state.clone()
        root_states[:, :3] = body_pos_w[time_step][:, 0] + scene.env_origins[:, None, :]
        root_states[:, 3:7] = body_quat_w[time_step][:, 0]
        root_states[:, 7:10] = body_lin_vel_w[time_step][:, 0]
        root_states[:, 10:] = body_ang_vel_w[time_step][:, 0]

        robot.write_root_state_to_sim(root_states)

        # Joint state
        robot.write_joint_state_to_sim(joint_pos[time_step], joint_vel[time_step])

        scene.write_data_to_sim()
        sim.render()
        scene.update(sim_dt)

        pos_lookat = root_states[0, :3].cpu().numpy()
        sim.set_camera_view(pos_lookat + np.array([2.0, 2.0, 0.5]), pos_lookat)


def main():
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device)
    sim_cfg.dt = args_cli.dt
    sim = SimulationContext(sim_cfg)

    scene_cfg = ReplayMotionsSceneCfg(num_envs=1, env_spacing=2.0)
    scene = InteractiveScene(scene_cfg)

    sim.reset()
    run_simulator(sim, scene)


if __name__ == "__main__":
    main()
    simulation_app.close()
