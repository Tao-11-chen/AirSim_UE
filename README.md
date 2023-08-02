# AirSim_UE
***Here is one of the simplest ways to get RGB images and depth maps from Unreal Engine using AirSim.***

In this method, you can get most of the information from rosbag and use CVMode to get high FPS images and depth maps.

## 1. Install Unreal Engine and compile AirSim for it
Please refer to https://www.unrealengine.com/en-US/download and https://microsoft.github.io/AirSim/ to install them.

For beginners, refer to https://zhuanlan.zhihu.com/p/619214564, which is a great Chinese tutorial.

**Tips:** 

Install it on Linux is not recommended because you can't create your Unreal Project there, it means you still need to use Windows to create the project and transfer it to the Linux system.

Install it on a remote server is not recommended because any remote image transmission experience rendered through GPU is not good as far as I know.

## 2. Install a version of WSL on your Windows and install ROS
This is the most convenient way to get information through ROS in Windows.

Refer to https://learn.microsoft.com/en-us/windows/wsl/install to install WSL with Ubuntu on Windows.

Refer to http://wiki.ros.org/ROS/Installation to install ROS in Ubuntu.

Then, remember to install the same version of AirSim in Ubuntu

## 3. Setup AirSim

**Now you have all the requirements installed, you need to set up the AirSim config file**

**GT.json** in this repo is used to record Lidar or GroundTruth Odom to rosbag.

**CVMode.json** in this repo is used to record images and depth maps.

**Here are the steps:**

1. copy GT.json to "C:\Users\User\Documents\AirSim" and rename it to "settings.json".

2. Set the GameMode in Unreal Engine to "AirSimGameMode" and run.

3. Run this in WSL. If failed, check your AirSim installation.
```console
roslaunch airsim_ros_pkgs airsim_node.launch
```

4. Now you should get this in WSL while running rostopic list:
```console
/airsim_node/drone_1/altimeter/barometer
/airsim_node/drone_1/environment
/airsim_node/drone_1/global_gps
/airsim_node/drone_1/gps/gps
/airsim_node/drone_1/imu/imu
/airsim_node/drone_1/magnetometer/magnetometer
/airsim_node/drone_1/odom_local_ned
/airsim_node/drone_1/vel_cmd_body_frame
/airsim_node/drone_1/vel_cmd_world_frame
/airsim_node/gimbal_angle_euler_cmd
/airsim_node/gimbal_angle_quat_cmd
/airsim_node/origin_geo_point
/rosout
/rosout_agg
/tf
/tf_static
```

## 4. Record your odom and other data to a rosbag


## 5. Setup AirSim to CV Mode


## 6. Using a Python script to get the images and depth maps you want


## 7. Extract other information from rosbag
