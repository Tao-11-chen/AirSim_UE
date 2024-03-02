# AirSim_UE
***Here is one of the simplest ways to get RGB images and depth maps from Unreal Engine using AirSim.(For ROS users please refer to branch ros)***

<img src="1.png" width="400" height="200">   <img src="2.jpg" width="400" height="200"> 
<img src="3.jpg" width="400" height="200">   <img src="4.jpg" width="400" height="200"> 

In this method, you can get most of the information including lidar and GroundTruth trajectory from rosbag and use CVMode to get high FPS(whatever you want) images and depth maps.

## 1. Install Unreal Engine and compile AirSim for it
Please refer to https://www.unrealengine.com/en-US/download and https://microsoft.github.io/AirSim/ to install them.

For beginners, refer to https://zhuanlan.zhihu.com/p/619214564, which is a great Chinese tutorial.

Then you need to find a good scene for your Unreal Engine.

**Tips:** 

Install it on Linux is not recommended because you can't create your Unreal Project there, it means you still need to use Windows to create the project and transfer it to the Linux system.

Install it on a remote server is not recommended because any remote image transmission experience rendered through GPU is not good as far as I know.

## 2. Setup AirSim

**Now you have all the requirements installed, you need to set up the AirSim config file**

**GT.json** in this repo is used to record Lidar or GroundTruth Odom to rosbag.

**CVMode.json** in this repo is used to record images and depth maps.

**Here are the steps:**

1. Copy GT.json to "C:\Users\User\Documents\AirSim" and rename it to "settings.json".

2. Set the GameMode in Unreal Engine to "AirSimGameMode" and run.

## 4. Record your odom using python

1.Using python API to control your drone or vehicle, "simple.py" and "fly_circle.py" in the folder are two examples.

2.Run the python file to record the trajectory:
```console
rosbag record -O bag_name.bag /airsim_node/drone_1/odom_local_ned
```

## 5. Setup AirSim to CV Mode

**GT.json** in this repo is used to record Lidar or GroundTruth Odom to rosbag.

**CVMode.json** in this repo is used to record images and depth maps.

1. Rename current "settings.json" to "GT.json"
  
2. Copy CVMode.json to "C:\Users\User\Documents\AirSim" and rename it to "settings.json"

3. Rerun Unreal Engine, now airsim is running in CVMode

## 6. Using a Python script to get the images and depth maps you want
Run the script get_img_using_rosbag.py with config args. Here is an example:

```console
python3 get_img_using_rosbag.py --bag bag_name.bag --dir '/mnt/d/mydata'
```


## Contact me

yuantao@xauat.edu.cn


