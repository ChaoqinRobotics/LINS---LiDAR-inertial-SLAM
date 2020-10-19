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

We provide a short piece of dataset for testing. The dataset can be founded [here](https://drive.google.com/file/d/19UUcO77L-g-RsZd_SLyr6O39S2JXJtiK/view?usp=drive_web) or [here](https://drive.google.com/file/d/19UUcO77L-g-RsZd_SLyr6O39S2JXJtiK/view?usp=sharing)

1. `source devel/setup.bash`
2. `roslaunch lins run_port_exp.launch`.
3. `rosbag play lidar_imu_dataset.bag --clock`.



## Run Your Own Dataset

Before you run your own dataset, please make sure that the extrinsic parameters between LiDAR and IMU are estimated off-line and set in the config file (see exp_port.yaml). In this version of LINS, we assume the roll and pitch angles between LiDAR and IMU are zero, and users can set the yaw angles between them by

```cpp
imu_misalign_angle
```

The IMU noice parameters should be estimted and set in this file, too.

```cpp
# noice parameters
acc_n: 70000
gyr_n: 0.1
acc_w: 500
gyr_w: 0.05
    
# initial IMU biases
init_ba: !!opencv-matrix
   rows: 3
   cols: 1
   dt: d
   data: [-0.015774,0.143237,-0.0263845]
   
init_bw: !!opencv-matrix
   rows: 3
   cols: 1
   dt: d
   data: [-0.00275058,-0.000165954,0.00262913]
```

Below is the sensor we use in data collection.

![sensor set](https://github.com/ChaoqinRobotics/LINS---LiDAR-inertial-SLAM/blob/master/sensor/fig_lidar.png)

Because this sensor set has small translation and rotation between LiDAR and IMU, the extrinsic parameters in the config file can be set to zero. **But this may not be the case in your sensor sets.**

Also, in this version of LINS, please make sure that the vehicle starts at the flat road, i.e., the roll and pitch angles are close to zero.



## Cite *LINS*

Thank you for citing our LINS paper if you use any of this code:

```
@inproceedings{qin2020lins,
  title={LINS: A Lidar-Inertial State Estimator for Robust and Efficient Navigation},
  author={Qin, Chao and Ye, Haoyang and Pranata, Christian E and Han, Jun and Zhang, Shuyang and Liu, Ming},
  booktitle={2020 IEEE International Conference on Robotics and Automation (ICRA)},
  pages={8899--8906},
  year={2020},
  organization={IEEE}
}
```



