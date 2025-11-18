# Project Updates from Cam

This document lists the current status of issues and tasks for the slide_slam hercules integration project.

## Generating rosbags
- Use the to_bag.py function in robotdataprocess (examples/Hercules_V1.4) with the paths to the directories for each type changed to fit dataset
- In a python environment, pip install rosbags. Use the rosbags-convert functionality to convert from ROS2 to ROS1. Documentation found online

# General Status
- Hercules trees are being segmented and point clouds associated with each tree are being published.
- The point cloud processing node is receiving these point clouds and attempting to hand them to the cylinder modeler.
- The cylinder modeler is not able to fit cylinders to these point cloud messages.

# Issues/Tasks by Node

## Semantic Segmentation Node (hercules_detect_open_vocab.py)
**Enhancements**:
- Currently uses yolo_world model tuned with additional tree training. Could be exchanged for a truly open vocab model such as fastsam in the future.

## Point Cloud Processing Node (hercules_process_cloud_node.py)
**Issues**:
- Currently does not extract a ground plane. This either needs to be fixed for the cylinder modeler to work or change how the cylinder modeler fits trees.
- Bug causing hercules_process_cloud_node to throw an error some time after launch. Does not seem associated with a specific trigger other than time.

**Next Step**:
- Start by figuring out the ground plane issue.

## Cylinder Modeler
**Issues**:
- Thinks that it does not have enough points from a tree point cloud to fit a cylinder
- Does not have access to ground plane. It assumes trees are perpendicular to ground plane.
- Parameters in configuration files not tuned to trees from hercules dataset.

**Next Step**:
- Start with fixing ground plane problem or trying to model cylinders without a ground plane

## Other Issues/Notes
- TF Tree is not updated for hercules dataset. Everything breaks when I tried to assign new frames associated with hercules.
- Many files from SLIDE_SLAM were not needed for this project. They are stored in the "excess" directory.
- Have the original SLIDE_SLAM repository on-hand in case changes need to be reverted.