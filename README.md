# Reinforcement training for hightorque minipi via video input (.mp4)
## Overview
This repository adapts the GVHMR → GMR → BeyondMimic pipeline for half-body humanoid robotics. It should be noted that due to the half-body nature of robot, certain motions that require upper body to balance may be difficult to reproduce.

The frameworks takes an input video of human motion and:
- [(GVHMR)](https://github.com/zju3dv/GVHMR) reconstructs full-body 3D human motion (pose and global trajectory) from input video.
- [(GMR)](https://github.com/YanjieZe/GMR) retargets the reconstructed human motion into robot joint motions, producing a trajectory the robot can track.
- [(BeyondMimic)](https://github.com/HybridRobotics/whole_body_tracking) trains and runs an RL tracking policy so the humanoid robot follows the retargeted motion in simulation.

This repository uses [hightorque_minipi](https://www.hightorquerobotics.com/pi/#) as robot model but can be adapted to other half robots. Any changes made to the original code will be documented as reference in the ['Changes & Notes'](#changes--notes) section to support other half body robots.

## Import repository
how to import code...

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

<p align="left">
  <img src="exercise/demo.gif" width="400">
</p>

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
python scripts/gvhmr_to_robot.py
--robot hightorque_minipi
--gvhmr_pred_file ~RL-minipi/GVHMR/outputs/demo/{exercise}/hmr4d_results.pt
--record_video   --rate_limit
--save_path outputs/demo/{exercise}.csv
```

The output `.csv` will be saved to `GMR/outputs/demo` and video saved to `GMR/videos`.


## BeyondMimic
> [!NOTE]
> The code is tested on Ubuntu 22.04, using IsaacSim 5.1 and IsaacLab V2.1.0


[BeyondMimic](https://github.com/HybridRobotics/whole_body_tracking)

Copy the .csv output from GMR first into the `whole_body_tracking` folder.
```bash
cp 

```


```bash
cd RL-minipi/whole_body_tracking

```


---
## Changes & Notes
### GVHMR
No changes were made to the original repository.

### GMR

1. Import robot's urdf + meshes (.stl) files into `GMR/assets` folder.
   - The urdf file has to include a floating point in GMR:
  ```bash
	<link name="world_link"/>

		<joint name="root_joint" type="floating">
		  <parent link="world_link"/>
		  <child link="base_link"/>
		  <origin xyz="0 0 0" rpy="0 0 0"/>
		</joint>
  ```

2. Create `smplx_to_hightorque_minipi.json` in `general_motion_retargeting/ik_configs`
```bash
"base_link": ["pelvis", 100, 10, [0,0,0], [0.5,-0.5,-0.5,-0.5]]
"robot_frame_name": ["human_body_name", pos_weight, rot_weight, pos_offset, rot_offset_quat]
```

3. Edit `general_motion_retargeting/param.py`
   
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


#### Changes
### BeyondMimic



## Third-party projects (credits)

This repository provides MiniPi-specific integration code and configs and depends on the following open-source projects:

- **GVHMR** (used for human motion reconstruction): https://github.com/zju3dv/GVHMR  
- **GMR** (used for retargeting): https://github.com/YanjieZe/GMR (MIT License)
- **whole_body_tracking / BeyondMimic** (used for RL tracking in IsaacLab): https://github.com/HybridRobotics/whole_body_tracking (MIT License)

These projects are installed separately and remain under their respective licenses. Users must comply with upstream licenses and any model/dataset licenses (e.g., SMPL/SMPL-X).
