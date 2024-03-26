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
