mb_pose_estimation
==================

A ROS package for model-based pose estimation and tracking with localized robots.

## Example of use

First create a description of the object to track inside the data/ folder the mb_pose_estimation package, e.g. table.cao and table.init. 

* table.cao. Describes the object 3D model in CAO format
```
V1
4
0 0 0
1.215 0 0
1.215 0.755 0
0.0 0.755 0
0
0
1
4 0 1 2 3
0

```

* table.init. Contains the list of vertices the user has to click to train the model:

```
4
0 0 0
1.215 0 0
1.215 0.755 0
0.0 0.755 0

```

Then place the robot looking towards the object and let it 'learn' the visual appearance. Assuming the robot is localized in the /map frame (replace the topic names by suitable ones):

```
rosrun mb_pose_estimation training_node --id table --camera /head_camera/image_rect --camera-info /head_camera/camera_info --world-frame /map
```

Click on the image on top of the vertices described in the .init file (in the same order). After this, the robot will store a set of descriptors that will help estimating the pose in the future.

Once your models have been trained, start the pose estimation service:

```
rosrun mb_pose_estimation pose_estimation_service --camera /head_camera/image_rect --camera-info /head_camera/camera_info --debug --world-frame map
```

Your can now request the pose of an object though a call to the /pose_estimation service. There is a client example:

```
rosrun mb_pose_estimation pose_estimation_client --id table
```
