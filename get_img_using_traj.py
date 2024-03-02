#!/usr/bin/env python3
'''
function: using rosbag with GT poses to fetch images from Unreal Engine
.pfm(depth) and .png(rgb)
'''
import airsim
import math
from cv_bridge import CvBridge
import time, sys, os
import argparse
import numpy as np

''' Reference
https://microsoft.github.io/AirSim/image_apis/
The cameras on the drone can be accessed by following names in API calls:
 front_center, front_right, front_left, bottom_center and back_center.
 For backward compatibility you can still use following ID numbers 
 for above camera names in same order as above: "0", "1", "2", "3", "4" 
'''

#setup the argument list
parser = argparse.ArgumentParser(description='Get images data from rosbag')
parser.add_argument('--traj', metavar='traj_file', default='./gt.txt', help='trajectory file name', required=True)
parser.add_argument('--dir', metavar='dir', default='/mnt/d/mydata', help='images dir to save data', required=True)

#print help if no argument is specified
if len(sys.argv) < 1:
    parser.print_help()
    sys.exit(0)

#parse the args
parsed = parser.parse_args()

try:
    client = airsim.VehicleClient()
    client.confirmConnection()

    # Creating floders
    dir = parsed.dir
    depth_dir = os.path.join(dir, "depth")
    rgb_dir = os.path.join(dir, "rgb")
    if not os.path.isdir(depth_dir):
        try:
            os.mkdir(depth_dir)
        except OSError as error:
            print("Image dir not available")
            sys.exit(0)
        os.mkdir(rgb_dir)

    # read odometry data
    file_name = parsed.traj
    vec_time = []
    vec_pose = []
    all_traj = np.loadtxt(fname=file_name)
    for traj in all_traj:
        vec_time.append(traj[0])
        vec_pose.append(traj[1:].tolist())

    # write images
    last_time = vec_time[0]/1000
    cur_time = vec_time[0]/1000
    isFirstFrame = True
    bridge_rgb = CvBridge()
    for i, pose in enumerate(vec_pose):
        if not isFirstFrame and ((cur_time - last_time) < 0.03333): # TODO: 30Hz image
            isFirstFrame = False
            cur_time = vec_time[i]/1000
        else:
            isFirstFrame = True
            last_time = cur_time
            cur_time = vec_time[i]/1000
            vehicle_pose = airsim.Pose(airsim.Vector3r(pose[0], pose[1], pose[2]), airsim.Quaternionr(pose[3], pose[4], pose[5], pose[6]))
            client.simSetVehiclePose(vehicle_pose, True)
            camera_pose = airsim.Pose(airsim.Vector3r(0, 0, 0), airsim.to_quaternion(math.radians(0), 0, math.radians(0))) # TODO: Give an extra rotation to camera
            # camera_pose = airsim.Pose(airsim.Vector3r(0, 0, 0), airsim.to_quaternion(0, 0, math.radians(60)))
            client.simSetCameraPose("0", camera_pose)
            time.sleep(0.3) # sleep 0.3 sec for engine prepare
            responses = client.simGetImages([
                # camera_name, image_type, pixels_as_float=False, compress=True
                airsim.ImageRequest("0", airsim.ImageType.Scene), 
                airsim.ImageRequest("0", airsim.ImageType.DepthPerspective, True)])
            dist_params = client.simGetCameraInfo(0)
            print(f"Updated Distortion Params: {dist_params}")
            for idx_img, response in enumerate(responses):
                if response.pixels_as_float:
                    print("Type %d, size %d" % (response.image_type, len(response.image_data_float)))
                    filename = os.path.join(depth_dir, str(cur_time))
                    airsim.write_pfm(os.path.normpath(filename + '.pfm'), airsim.get_pfm_array(response))
                else:
                    print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
                    filename = os.path.join(rgb_dir, str(cur_time))
                    airsim.write_file(os.path.normpath(filename + '.png'), response.image_data_uint8)
except:
    pass
