
## 1. Prerequisites
1.1 **Ubuntu** and **ROS**
Ubuntu  20.04.
ROS noetic. [ROS Installation](http://wiki.ros.org/ROS/Installation)
additional ROS pacakge
```
    sudo apt-get install ros-YOUR_DISTRO-cv-bridge ros-YOUR_DISTRO-tf ros-YOUR_DISTRO-message-filters ros-YOUR_DISTRO-image-transport
```


1.2. **Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html), remember to **make install**.
(Our testing environment: Ubuntu 20.04, ROS noetic, OpenCV 3.3.1, Eigen 3.3.3) 
1.3 **Install GTSAM**
```
sudo apt-get -y install libboost-all-dev
sudo apt-get -y install libtbb-dev
git clone https://github.com/borglab/gtsam.git
cd gtsam && git checkout 4.2a8
mkdir build
cd build
cmake -DGTSAM_USE_SYSTEM_EIGEN ..
make check (optional, runs unit tests)
make install
```
## 2. Build VINS-Mono on ROS
Clone the repository and catkin_make:
```
    cd ~/catkin_ws/src
    git clone [https://github.com/HKUST-Aerial-Robotics/VINS-Mono.git](https://github.com/Garuda-IIITH-RRC/VINS-Mono-GTSAM.git)
    cd VINS-Mono && git checkout origin/Gtsam_hydra_V2 && cd ..
    cd ../
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```

## 3. Visual-Inertial Odometry and Pose Graph Reuse on our datasets

#3.1 Our Hardware setup
    Intel NUC i7 processor
    D455 stereo camera


Suppose you are familiar with ROS and you can get a camera and an IMU with raw metric measurements in ROS topic, you can follow these steps to set up your device.

3.1.1 Change to your topic name in the config file (realsense_color_config.yaml). The image should exceed 20Hz and IMU should exceed 100Hz. Both image and IMU should have the accurate time stamp. IMU should contain absolute acceleration values including gravity.

3.2 Camera calibration:

We support the [pinhole model](http://docs.opencv.org/2.4.8/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html) and the [MEI model](http://www.robots.ox.ac.uk/~cmei/articles/single_viewpoint_calib_mei_07.pdf). You can calibrate your camera with any tools you like. Just write the parameters in the config file in the right format. If you use rolling shutter camera, please carefully calibrate your camera, making sure the reprojection error is less than 0.5 pixel.

3.3 **Camera-Imu extrinsic parameters**:

 you can find that we can estimate and refine them online. If you familiar with transformation, you can figure out the rotation and position by your eyes or via hand measurements. Then write these values into config as the initial guess. Our estimator will refine extrinsic parameters online. If you don't know anything about the camera-IMU transformation, just ignore the extrinsic parameters and set the **estimate_extrinsic** to **2**, and rotate your device set at the beginning for a few seconds. When the system works successfully, we will save the calibration result. you can use these result as initial values for next time. An example of how to set the extrinsic parameters is in[extrinsic_parameter_example](https://github.com/HKUST-Aerial-Robotics/VINS-Mono/blob/master/config/extrinsic_parameter_example.pdf)

3.4 **Temporal calibration**:
Most self-made visual-inertial sensor sets are unsynchronized. You can set **estimate_td** to 1 to online estimate the time offset between your camera and IMU.  

3.5 **Rolling shutter**:
For rolling shutter camera (carefully calibrated, reprojection error under 0.5 pixel), set **rolling_shutter** to 1. Also, you should set rolling shutter readout time **rolling_shutter_tr**, which is from sensor datasheet(usually 0-0.05s, not exposure time). Don't try web camera, the web camera is so awful.

3.6 Other parameter settings: Details are included in the config file.

3.7 Performance on different devices: 

(global shutter camera + synchronized high-end IMU, e.g. VI-Sensor) > (global shutter camera + synchronized low-end IMU) > (global camera + unsync high frequency IMU) > (global camera + unsync low frequency IMU) > (rolling camera + unsync low frequency IMU). 

## 4. Mapping module

 Vox blox mapping module is used to create mesh. Clone the git repo in to the source directory of your workspace
```
    git clone https://github.com/ethz-asl/voxblox
    cd ../
    catkin_make
    mv /src/support_file/d455_outdoor.launch /src/voxblox/voxblox_ros/launch #edit the launch file with your camera topic names
    source devel/setup.bash
```
Install all the dependencies vox blox requires to build.
4.1 Edit the launch file d455_outdoor.launch
    1. You can add the pointcloud topic in the “pointcloud remap” (/camera_d455/depth/color/points)
    2. You can change the world frame using the “world_frame param”. This will correspond to the map frame or the global frame
    3. You can change the sensor frame using the “sensor_frame param”. This will correspond to the sensor frame or the camera frame             from which the depth points are being published
    4. You can change the resolution of the map(size of voxels) or the mesh by changing the “tsdf_voxel_size param”
**5.1 visual-inertial odometry and loop closure**

5.1.1 Open three terminals, launch the vins_estimator , rviz and play the bag file respectively. 
```
    roslaunch vins_estimator realsense_color.launch 
    roslaunch vins_estimator vins_rviz.launch
    roslaunch voxblox_ros d455_outdoor.launch 
    rosbag play YOUR_PATH_TO_DATASET/your.bag 
```
(If you fail to open vins_rviz.launch, just open an empty rviz, then load the config file: file -> Open Config-> YOUR_VINS_FOLDER/config/vins_rviz_config.rviz)


**No extrinsic parameters** in that config file.  Waiting a few seconds for initial calibration. Sometimes you cannot feel any difference as the calibration is done quickly.

**5.2 map merge**

After playing 1 bag, you can continue playing 2 bag, 3 bag ... The system will merge them according to the loop closure.

**5.3 Mapping**

5.3.1 map save

To save map run the following command in a new terminal
```
rosservice call /voxblox_node/generate_mesh "{}"
```
The mesh is saved in voxblox_ros/mesh_results


