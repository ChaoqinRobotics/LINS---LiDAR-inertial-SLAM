# LINS---LiDAR-inertial-SLAM

This repository contains code for a tightly-coupled lidar-inertial odometry and mapping system for ROS compatible UGVs. The reason of fusing IMU and Lidar in a tightly-couple scheme is to handle feature-less environments where previous methods may fail. This work is built upon LIO-mapping, LeGO-LOAM and LOAM. The main contribution of this work is the lightweight lidar-inertial lidar odometry which produces robust and accurate odometry in real time using a 400-Hz IMU and a Velodyne VLP-16 Lidar. Extensive experiments show that with a robust lidar-inertial odometry, the localization and mapping performances are greatly improved in challenging environments. A demonstration of the system can be found here -> https://www.youtube.com/watch?v=Nmr1blC09qw&t=8s



## Dependency

- [ROS](http://wiki.ros.org/ROS/Installation) (tested with kinetic)
- [gtsam](https://github.com/borglab/gtsam/releases) (Georgia Tech Smoothing and Mapping library)
- [OpenCV](https://opencv.org/) (tested with OpenCV 3.4)


## Compile

You can use the following commands to download and compile the package.

```
cd ~/catkin_ws/src
git clone https://github.com/ChaoqinRobotics/LINS---LiDAR-inertial-SLAM.git
cd ..
catkin_make -j1
```

When you compile the code for the first time, you need to add "-j1" behind "catkin_make" for generating some message types. "-j1" is not needed for future compiling.



## The system

Like LeGO-LOAM, LINS is speficifally optimized for a horizontally placed VLP-16 on a ground vehicle with a high-speed IMU (>=100Hz). It assumes there is always a ground plane in the scan. 



## Run the package

1. Run the launch file:

```
roslaunch lins run_port_exp.launch
```

Notes: The parameter "/use_sim_time" is set to "true" for simulation, "false" to real robot usage.

1. Play existing bag files:

```
rosbag play *.bag --clock --topic /velodyne_points /imu/data
```



## Dataset

We provide a short piece of dataset for testing. The dataset can be founded [here](https://drive.google.com/file/d/19UUcO77L-g-RsZd_SLyr6O39S2JXJtiK/view?usp=drive_web).

1. `source devel/setup.bash`
2. `roslaunch lins run_port_exp.launch`.
3. `rosbag play lidar_imu_dataset.bag --clock`.

## Cite *LINS*

Thank you for citing our LINS paper if you use any of this code:

```

```
