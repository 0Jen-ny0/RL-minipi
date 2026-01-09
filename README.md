# Reinforcement training for hightorque minipi via video input (.mp4)
## Overview
This repository adapts the GVHMR → GMR → BeyondMimic pipeline for a half-body humanoid robot (HighTorque MiniPi). Because MiniPi has a reduced upper-body DoF, motions that rely heavily on upper-body balancing may be difficult to reproduce.

The framework takes an input video of human motion and:
- [(GVHMR)](https://github.com/zju3dv/GVHMR) reconstructs full-body 3D human motion (pose and global trajectory) from input video.
- [(GMR)](https://github.com/YanjieZe/GMR) retargets the reconstructed human motion into robot joint motions, producing a trajectory the robot can track.
- [(BeyondMimic)](https://github.com/HybridRobotics/whole_body_tracking) trains and runs an RL tracking policy so the humanoid robot follows the retargeted motion in simulation.

This repository uses [hightorque_minipi](https://www.hightorquerobotics.com/pi/#) as the robot model, but it can be adapted to other reduced-DoF humanoid platforms. All deviations from upstream projects are documented in the ['Changes & Notes'](#changes--notes) section.

| Sideway walk | Stomping |
|---|---|
| <img src="exercise/demo/gifs/sideway_walk.gif" width="350"> | <img src="exercise/demo/gifs/stomping.gif" width="350"> |

To import repository:
```bash
git clone https://github.com/0Jen-ny0/RL-minipi.git
```
---
## GVHMR

> [!NOTE]
> Tested on Ubuntu 22.04.

###  Setup
> Please refer to the [GVHMR](https://github.com/zju3dv/GVHMR) repository for environment setup and usage. This repository does not redistribute GVHMR code; please follow the upstream installation instructions.

An example package `exercise` is provided in this repository. Download it and place it under `GVHMR/outputs/demo/{exercise}`.

### Run GVHMR

To process the motion video:
```bash
cd GVHMR
python tools/demo/demo.py --video=outputs/demo/{exercise}/{exercise}.mp4 -s
```
To play the output video:
```bash
ffplay outputs/demo/{exercise}/{exercise}_3_incam_global_horiz.mp4
```

Outputs are saved under `GVHMR/outputs/demo/{exercise}`, including the hmr4d file and the rendered preview video (example below).

| Exercise | Squat | Feet up |
|---|---|---|
| <img src="exercise/demo/gifs/exercise.gif" width="250"> | <img src="exercise/demo/gifs/squat.gif" width="250"> | <img src="exercise/demo/gifs/feetup.gif" width="250"> |

Full demo video: [watch here](https://github.com/user-attachments/assets/7cd875c9-f437-4607-ad3a-5d7dcecb2965)



## GMR
> [!NOTE]
> Tested on Ubuntu 22.04.
> This repository is based on [GMR](https://github.com/YanjieZe/GMR) and includes modifications to support the `hightorque_minipi` model.

### Setup
Create your conda environment:
```bash
conda create -n gmr python=3.10 -y
conda activate gmr
conda install -c conda-forge libstdcxx-ng -y
```

Download the [SMPL-X body](https://github.com/vchoutas/smplx) models and place them in `GMR/assets/body_models`, following this structure:
```bash
- GMR/assets/body_models/smplx/
-- SMPLX_NEUTRAL.pkl
-- SMPLX_FEMALE.pkl
-- SMPLX_MALE.pkl
```

### Run GMR
Go to the GMR directory:
```bash
cd RL-minipi/GMR
```

Run the command below to retarget the extracted human pose data to your robot:
```bash
python scripts/gvhmr_to_robot.py \
  --robot hightorque_minipi \
  --gvhmr_pred_file ~/RL-minipi/GVHMR/outputs/demo/{exercise}/hmr4d_results.pt \
  --record_video \
  --rate_limit \
  --save_path outputs/demo/{exercise}.csv
```

The output `.csv` will be saved to `GMR/outputs/demo` and video saved to `GMR/videos`.

| Squat | Feet up|
|---|---|
| <img src="exercise/demo/gifs/squat_gmr.gif" width="350"> | <img src="exercise/demo/gifs/feetup_gmr.gif" width="350"> |


## BeyondMimic
> [!NOTE]
> Tested on Ubuntu 22.04 with Isaac Sim 5.1 and Isaac Lab v2.1.0.

### Installation
> This repository is based on [BeyondMimic](https://github.com/HybridRobotics/whole_body_tracking) and includes modifications to support the `hightorque_minipi` model.

Install Isaac Sim and Isaac Lab and create the conda environment:
```bash
wget -O install_isaaclab.sh https://docs.robotsfan.com/install_isaaclab.sh && bash install_isaaclab.sh # One-line installation code
conda activate {environment}
```
Or install specific version of [Isaac Sim](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/download.html) and [Isaac Lab](https://isaac-sim.github.io/IsaacLab/v2.1.0/source/setup/installation/index.html).




### Run BeyondMimic

#### Pre-processing + WandB setup

Copy the .csv output from GMR into the `whole_body_tracking` folder.
```bash
cp ~/RL-minipi/GMR/outputs/demo/*.csv ~/RL-minipi/whole_body_tracking/
cd RL-minipi/whole_body_tracking
conda activate {environment}
python -m pip install -e source/whole_body_tracking
```

In [Weights & Biases](https://wandb.ai/home), open Registry and create a new collection named Motions (artifact type: All Types).

Convert csv into npz and upload into WandB:
```bash
python scripts/csv_to_npz.py --input_file {exercise}.csv --input_fps 30 --output_name {exercise} --headless
```

To replay the uploaded motion:
```bash
python scripts/replay_npz.py --registry_name={your-organization}--org/wandb-registry-motions/{exercise}
```


#### Policy training
Train policy by the following command:
```bash
python scripts/rsl_rl/train.py \
  --task=Tracking-Flat-{MiniPi}-v0 \
  --registry_name={your-organization}--org/wandb-registry-motions/{exercise}:latest \
  --headless \
  --logger=wandb \
  --log_project_name=whole-body-tracking \
  --run_name={exercise}
```
Play the trained policy by the following command:
```bash
python scripts/rsl_rl/play.py \
  --task=Tracking-Flat-{MiniPi}-v0 \
  --num_envs=2 \
  --wandb_path={wandb-run-path} \
  --video \
  --video_length={desired_length}
```

| Sideway walk |
|---|
| <img src="exercise/demo/gifs/sideway_walk.gif" width="600"> |

| Stomping |
|---|
| <img src="exercise/demo/gifs/stomping.gif" width="600"> |

---
## Changes & Notes
### GVHMR
No changes were made to the original repository.

### GMR

1. Import robot's urdf + meshes (.stl) files into `GMR/assets` folder.
   - The urdf file must include a floating root joint in GMR:
  ```bash
	<link name="world_link"/>

		<joint name="root_joint" type="floating">
		  <parent link="world_link"/>
		  <child link="base_link"/>
		  <origin xyz="0 0 0" rpy="0 0 0"/>
		</joint>
  ```

2. Create `smplx_to_{robot_model}.json` in `general_motion_retargeting/ik_configs`
```bash
# Example format
"base_link": ["pelvis", 100, 10, [0,0,0], [0.5,-0.5,-0.5,-0.5]]
"robot_frame_name": ["human_body_name", pos_weight, rot_weight, pos_offset, rot_offset_quat]
```

3. Edit `general_motion_retargeting/params.py`
   
```bash
Add the name of the robot model where # Add robot model
```

4. `gvhmr_ro_robots.py`
   - Add "hightorque_minipi" or other models into 'choices'
   - Update the correct joint list into 'motion_data' and 'qpos_list'

Note that GMR processes using rotation order (w-x-y-z), but input of BeyondMimic requires rotation order (x-y-z-w), so the output format of .csv file is altered:
  ```bash
csv file context
            root_pos=qpos[:3],
            root_rot=qpos[3:7],
            dof_pos=qpos[7:],
# refer to the robot's urdf file for joint names and order 
  ```

### BeyondMimic
1.Import robot's urdf file into `source/whole_body_tracking/whole_body_tracking/assets`.

Remove the floating joint in the urdf:
```bash
# Remove something like this
	<link name="world_link"/>
		<joint name="root_joint" type="floating">
		  <parent link="world_link"/>
		  <child link="base_link"/>
		  <origin xyz="0 0 0" rpy="0 0 0"/>
		</joint>
```

2.Create `{robot_model}.py` in `source/whole_body_tracking/whole_body_tracking/robots`

```bash
# Update ARMATURE + velocity according to the robot's specification
# Armature = (J_rotor + J_reducer)* 1e-6 * N^2 , N = gear ratio
# Replace all variables with robot model name (Ctrl+F:minipi)
# Update the joint variables
```
**3. Edit `my_on_policy_runner` in `source/whole_body_tracking/whole_body_tracking/tasks/tracking/mdp` to adjust reward terms, logging, and export behavior.**
```bash
# Update rewards and etc to change policy optimisation
```
4.Copy + rename the `minipi` folder to {robot_model} in `source/whole_body_tracking/whole_body_tracking/tasks/tracking/config` and edit `flat_env_cfg.py`
```bash
# Replace all variables with robot model name (Ctrl+F:minipi)
# Update 'self.commands.motion.body_names'
```
5.Edit `tracking_env_cfg` in `source/whole_body_tracking/whole_body_tracking/tasks/tracking`
```bash
# Modify 'self.episode_length_s' to desired length
```

6.Edit `csv_to_npz.py` in `scripts`
```bash
# Replace all variables with robot model name (Ctrl+F:minipi)
# Replace the 'joint_names' (same as gvhmr)
```

7.Edit `replay_npz` in `scripts`
```bash
# Replace all variables with robot model name (Ctrl+F:minipi)
```

## Third-party projects (credits)

This repository provides MiniPi-specific integration code and configs and depends on the following open-source projects:

- **GVHMR** (used for human motion reconstruction): https://github.com/zju3dv/GVHMR  
- **GMR** (used for retargeting): https://github.com/YanjieZe/GMR (MIT License)
- **whole_body_tracking / BeyondMimic** (used for RL tracking in IsaacLab): https://github.com/HybridRobotics/whole_body_tracking (MIT License)

These projects are installed separately and remain under their respective licenses. Users must comply with upstream licenses and any model/dataset licenses (e.g., SMPL/SMPL-X).
