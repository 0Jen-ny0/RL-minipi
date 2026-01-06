# Reinforcement training for hightorque minipi via video input (.mp4)
## Overview
This repository adapts the GVHMR → GMR → BeyondMimic pipeline for half-body humanoid robotics. It should be noted that due to the half-body nature of robot, certain motions that require upper body to balance may be difficult to reproduce.

The frameworks takes an input video of human motion and:
- [(GVHMR)](https://github.com/zju3dv/GVHMR) reconstructs full-body 3D human motion (pose and global trajectory) from input video.
- [(GMR)](https://github.com/YanjieZe/GMR) retargets the reconstructed human motion into robot joint motions, producing a trajectory the robot can track.
- [(BeyondMimic)](https://github.com/HybridRobotics/whole_body_tracking) trains and runs an RL tracking policy so the humanoid robot follows the retargeted motion in simulation.

This repository uses [hightorque_minipi](https://www.hightorquerobotics.com/pi/#) as robot model but can be adapted to other half robots. Any changes made to the original code will be documented as reference in the ['Changes & Notes'](#changes--notes) section to support other half body robots.

---
## GVHMR

Please refer to the [GVHMR](https://github.com/zju3dv/GVHMR) repository for environment setup and usage. 

An example can be downloaded [here]() and imported into `GVHMR/outputs/demo/{exercise}` folder

```bash
cd GVHMR
python tools/demo/demo.py --video=outputs/demo/{exercise}/{exercise}.mp4 -s
```
To play the outputted video:
```bash
ffplay outputs/demo/{exercise}/{exercise}_3_incam_global_horiz.mp4
```

The output will be in the `GVHMR/outputs/demo/{exercise}` folder, including a hmr4d file and video shown below.

<p align="center">
  <img src="exercise/_demo.gif" width="420">
</p>

Full demo video: [watch here](https://github.com/user-attachments/assets/7cd875c9-f437-4607-ad3a-5d7dcecb2965)



## GMR
[GMR](https://github.com/YanjieZe/GMR)

## BeyondMimic
[BeyondMimic](https://github.com/HybridRobotics/whole_body_tracking)

---
## Changes & Notes
### GVHMR
No changes were made to the original repository.
### GMR

#### Changes
### BeyondMimic



## Third-party projects (credits)

This repository provides MiniPi-specific integration code and configs and depends on the following open-source projects:

- **GVHMR** (used for human motion reconstruction): https://github.com/zju3dv/GVHMR  
- **GMR** (used for retargeting): https://github.com/YanjieZe/GMR (MIT License)
- **whole_body_tracking / BeyondMimic** (used for RL tracking in IsaacLab): https://github.com/HybridRobotics/whole_body_tracking (MIT License)

These projects are installed separately and remain under their respective licenses. Users must comply with upstream licenses and any model/dataset licenses (e.g., SMPL/SMPL-X).
