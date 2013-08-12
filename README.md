mb_pose_estimation
==================

A ROS package for model-based pose estimation and tracking with localized robots. See this video for an example:

https://www.youtube.com/watch?v=32SIoHGDzTE

## How to use

First create a description of the object to track inside the data/ folder the mb_pose_estimation package, e.g. tv.cao and tv.init. 

* tv.cao. Describes the object 3D model in CAO format (http://www.irisa.fr/lagadic/visp/documentation/visp-2.8.0/classvpMbTracker.html#a68b9020b5f30677f0b0fd17640292ae7)

```
V1
4
0 0 0
0.995 0 0
0.995 0.62 0
0.0 0.62 0
0
0
1
4 0 1 2 3
0

```

* tv.init. Contains the list of vertices the user has to click to train the model:

```
4
0 0 0
0.995 0 0
0.995 0.62 0
0.0 0.62 0

```

Then place the robot looking towards the object and let it 'learn' the visual appearance. Assuming the robot is localized in the /map frame (replace the topic names by suitable ones):

```
rosrun mb_pose_estimation training_node --id tv --camera /head_camera/rgb/image_rect --camera-info /head_camera/rgb/camera_info --world-frame /map
```

Click on the image on top of the vertices described in the .init file (in the same order), and, if the initial estimation looks good for you, click on the image with the mouse left button. After this, the robot will store a set of descriptors that will help estimating the pose in the future.

Once your model has been trained, start the pose estimation service:

```
rosrun mb_pose_estimation pose_estimation_service --camera /head_camera/rgb/image_rect --camera-info /head_camera/rgb/camera_info --debug --world-frame map
```

Your can now request the pose of an object from different robot locations (assuming the object appears in the image) trhough a call to the /pose_estimation service. There is a client example:

```
rosrun mb_pose_estimation pose_estimation_client --id tv
```
