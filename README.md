# origami-arm-sim
This repository contains a ROS package for simulating an origami inspired continuum manipulator and an RGB camera.


## Launch the simulator
Use the following launch command to launch the simlator and visualization
```roslaunch origami_sim sim.launch```


## sim_FK.py
To get the points on the robot or the "robot shape" in the image, use the following topic:
```origami_vs/robot_shape```

To get the points on the robot in Cartesian space you can write your own rostopic publisher in sim_FK.py. The object "points_3D" can be published for this.

## simulation_control_interface.py
To command cable lengths or cable velocities to the robot publish to the following ROS topic:
```origami_vs/d_cable_length```

To get the current cable lengths of the robot subscribe to the following ROS topic:
```origami_vs/cable_lengths```

## origami_sim_vis.py
To get the camera image subscribe to the following topic using image_view package in ROS:
```origami_vs/camera_image```

## config.yaml
motor_conversion: lumped parameter for setting a gear ratio and spool diameter on the motor
control_rate: frequency of the control loop as well as the camera publisher frequency
lam: visual servo gain
num_of_modules: initializes the number of origami modules to use
init_cable_lengths: the initial cable lengths for each module. Each module should have 3 cable lengths specified.
points_per_segment: "Resolution" of the robot curve in the image. Keep it to a low number to prevent the visualization from lagging behind the controller


## Simulation setup (details on camera frame and robot frame)
The camera is spawned (as the world frame) and robot is spawned on the camera/world frame.

The robot frame is translated along camera frame to ensure the robot's workspace is visible in the image
1. -200 along y
2.  200 along z

Two consecutive rotations are performed to the robot frame: 
1. -pi/2 rotation along X-axis
2.  pi/2 rotation along Z-axis

This makes it so that L1 is on the left and L2, L3 make a line parallel to camera-Z or perpendicular to image frame (similar to how the robot hardware is setup in lab)

Now we have a complete eye-to-hand setup in simulation. [cv2().projectpoints](https://docs.opencv.org/3.4/d9/d0c/group__calib3d.html) is used to project 3D points along the robot's body onto the image frame. Since L2 and L3 on the robot make a linke parallel to camera Z-axis, we can now constrain these two cables (in software) to operate together essentially constraining the robot to 2 Degrees of Freedom and a planar motion in the image.

<p align="center">
<img src="media\origami_robot_setup.jpeg" width="1280" height="960" />
</p>
