# TODO List

## 2024.10.1

- [x] dual_bottles_pick_easy
- [x] block_hammer_beat

## 2024.10.2

- [x] Modify the code generation logic and include multiple iterations.

## Models
1. Gripper Module: Return a suitable gripper grasping posture based on the axis and point calibrated in the object file.
2. Evaluation Module: Assess whether the gripper's posture is reasonable.
3. Handover Module: Hand over the objects on the left and right grippers to the other gripper with a suitable posture.
4. Motion Module: Move the gripper to perform the target action.
5. 

## API list

1. `get_gripper_pose_to_grab_object()`: Returns the appropriate gripper pose for grasping an object, where the grasping pose is obtained based on the object's calibration information.
2. `evaluate_gripper_pose()`: An evaluation function for gripper poses, which determines whether a given pose is suitable for the gripper.
3. ``