# RoboMantis: Dual 7‑DOF Robot Arms with YOLOv11‑Seg + FoundationPose Vision‑Based Grasping

<div align="center">
  <video width="480" controls>
    <source src="PASTE_DRAGGED_VIDEO_LINK_HERE" type="video/mp4">
  </video>
  <p>RoboMantis (Dual 7‑DOF robot arms)</p>
</div>

A dual 7‑degree‑of‑freedom robotic arm platform. Each arm has an iron
hex‑socket (Allen key) end‑effector. Workpieces have magnets embedded
inside, so the arms pick them up by magnetic attraction on contact.
Vision‑based grasping uses YOLOv11‑Seg for instance segmentation and
FoundationPose for 6D pose estimation, with dual‑arm coordinated motion
planning via MoveIt 2.

## Hardware
| Component | Details |
|---|---|
| Robot Arms | Dual 7‑DOF custom arms (RoboMantis) |
| Joint Motors | Robstride Dynamics: Left arm: RS01, RS03, 5× RS00; Right arm: RS06, RS03, 5× RS00 |
| End‑effectors | Iron hex socket (Allen key) ×2; magnets embedded in workpieces |
| RGB‑D Camera | Orbbec Gemini 305 |
| CAN Adapter | CANdle / gs_usb compatible device |
| Power Supply | 48V DC |
| Host | Ubuntu 22.04+x86_64, CUDA 12.1 |

## Zero‑Torque Mode
Gravity‑compensated free‑drag mode for kinesthetic teaching and VLA data
collection on both arms. Implemented as `ros2_control` controllers using
Pinocchio RNEA for gravity compensation plus adaptive damping (Kp=0, so
motors do not resist manual movement).
```bash
# 1. Real robot arms + MoveIt 2 + RViz (all‑in‑one launch)
ros2 launch robomantis_moveit_config dual_real_robot.launch.py

# 2. Zero‑torque mode
# For left arm
ros2 control switch_controllers --deactivate left_joint_trajectory_controller  --activate left_zero_torque_controller
# For right arm
ros2 control switch_controllers --deactivate right_joint_trajectory_controller --activate right_zero_torque_controller
# For both arms
ros2 control switch_controllers --deactivate left_joint_trajectory_controller right_joint_trajectory_controller --activate left_zero_torque_controller right_zero_torque_controller

# Return to normal trajectory control
# For left arm
ros2 control switch_controllers --deactivate left_zero_torque_controller --activate left_joint_trajectory_controller
# For right arm
ros2 control switch_controllers --deactivate right_zero_torque_controller --activate right_joint_trajectory_controller
# For both arms
ros2 control switch_controllers --deactivate left_zero_torque_controller right_zero_torque_controller --activate left_joint_trajectory_controller right_joint_trajectory_controller
```

## System Pipeline
```
RGB‑D Camera
     │
     ▼
YOLOv11‑Seg ──► object mask
     │
     ▼
FoundationPose ──► 6D pose (camera frame)
     │
     ▼
Hand‑eye transform ──► 6D pose (base_link frame)
     │
     ▼
MoveIt 2 (dual‑arm planning) ──► motion plan & execute
     │
     ▼
Iron end‑effectors contact magnet‑embedded workpieces → magnetic pickup
```

## Reference Interfaces
The screenshots below are provided as a visual reference for the expected
appearance of the FoundationPose pose estimation and easy_handeye2 calibration
interfaces.

<table>
  <tr>
    <td align="center">
      <img src="./assets/foundationpose.png" width="420" />
      <br /><strong>FoundationPose</strong> — 6D pose estimation
    </td>
    <td align="center">
      <img src="./assets/handeye.png" width="420" />
      <br /><strong>Hand‑eye Calibration</strong> — easy_handeye2 (eye‑on‑base)
    </td>
  </tr>
</table>

**FoundationPose** — 6D pose estimation
**Hand‑eye Calibration** — easy_handeye2

## Repository Structure
```
RoboMantis/
├── robomantis_description/      # URDF/Xacro, STL meshes, display launch (dual‑arm)
├── robomantis_hw_rs01/         # Left‑arm ros2_control hardware, CAN driver, zero‑torque controller(C++)
├── robomantis_hw_rs06/         # Right‑arm ros2_control hardware, CAN driver, zero‑torque controller(C++)
├── robomantis_moveit_config/    # MoveIt 2 config (dual‑arm SRDF, OMPL, controllers)
├── easy_handeye2/               # hand‑eye calibration (eye‑on‑base, fixed external camera)
└── easy_handeye2_msgs/          # calibration message definitions
```

## Environment
- Ubuntu 22.04+x86_64
- ROS2 Humble (Python 3.10)
- SocketCAN · CAN 2.0 Extended Frame · 1 Mbps
- CUDA 12.1
- PyTorch 2.1.0, torchvision 0.16.0, torchaudio 2.1.0
- Ultralytics (YOLOv11)
- [FoundationPose](https://github.com/NVlabs/FoundationPose)
- OrbbecSDK_ROS2 driver for Gemini 305
- Pinocchio (pip install pinocchio) — FK/IK/Gravity compensation

## Notes
- **YOLOv11‑Seg model**: The trained weights (`*.pt`) are **not** included in this
  repository. You need to collect your own RGB‑D images of the workpieces, annotate
  them with segmentation masks (e.g. using LabelMe), convert to YOLO format,
  and train the model yourself with Ultralytics.
- **FoundationPose**: Requires the target object's 3D mesh model as input. Prepare the
  mesh (e.g. from CAD / SolidWorks) and place it in the FoundationPose assets directory.
- **Hand‑eye calibration**: Must be performed once for your specific fixed‑camera mounting
  position using `easy_handeye2` (eye‑on‑base mode) before grasping can work.
  Calibrate each arm's `base_link` relative to the camera frame.
- **Dual‑arm grasp strategy node**: This repository provides the robot control, MoveIt 2
  planning, and low‑level motion execution for both arms. The high‑level grasp strategy
  node — which integrates YOLOv11‑Seg + FoundationPose, converts the estimated 6D pose
  from camera frame to each arm's `base_link` frame via hand‑eye calibration, and
  coordinates dual‑arm grasp execution — is **not** included and must be implemented by
  the user according to their specific setup and workflow.
- **Two Python environments**: The ROS2 packages run on system Python 3.10,
  while YOLOv11‑Seg and FoundationPose run in the `foundationpose` conda
  environment (Python 3.9). Do not mix them — install deep learning dependencies
  only in the conda environment, and build ROS2 packages with the system Python.

## Build
```bash
mkdir -p ~/RoboMantis_ws/src && cd ~/RoboMantis_ws/src
git clone https://github.com/nanj‑robotics/RoboMantis.git
cd ~/RoboMantis_ws
rosdep install --from‑paths src --ignore‑src -r -y
colcon build --symlink‑install
source install/setup.bash
```

## References
- FoundationPose: https://github.com/NVlabs/FoundationPose
- easy_handeye2: https://github.com/marcoesposito1988/easy_handeye2
- Robstride Dynamics: https://github.com/RobStride/EDULITE_A3
- Orbbec: https://github.com/orbbec/OrbbecSDK_ROS2
