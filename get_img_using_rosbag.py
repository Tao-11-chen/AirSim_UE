#!/usr/bin/env python3
'''
function: using rosbag with GT poses to fetch images from Unreal Engine
.pfm(depth) and .png(rgb)
'''
import airsim
import rosbag
import math
from cv_bridge import CvBridge
import time, sys, os
import argparse

''' Reference
https://microsoft.github.io/AirSim/image_apis/
The cameras on the drone can be accessed by following names in API calls:
 front_center, front_right, front_left, bottom_center and back_center.
 For backward compatibility you can still use following ID numbers 
 for above camera names in same order as above: "0", "1", "2", "3", "4" 
'''

#setup the argument list
parser = argparse.ArgumentParser(description='Get images data from rosbag')
parser.add_argument('--bag', metavar='bag', default='./mybag.bag', help='ROS bag file name', required=True)
parser.add_argument('--dir', metavar='dir', default='/mnt/d/mydata', help='images dir to save data', required=True)
parser.add_argument('--topic', metavar='topic', default='/airsim_node/drone_1/odom_local_ned', help='GT Topic in rosbag')

#print help if no argument is specified
if len(sys.argv) < 1:
    parser.print_help()
    sys.exit(0)

#parse the args
parsed = parser.parse_args()

try:
    client = airsim.VehicleClient()
    client.confirmConnection()

    # open the bag
    bag = rosbag.Bag(parsed.bag, "r")
    dir = parsed.dir
    depth_dir = os.path.join(dir, "depth")
    rgb_dir = os.path.join(dir, "rgb")
    try:
        os.mkdir(depth_dir)
    except OSError as error:
        print("Image dir not available")
        sys.exit(0)
    os.mkdir(rgb_dir)

    # read odometry data
    vec_time = []
    vec_pose = []
    for topic, msg, t in bag.read_messages(topics=[parsed.topic]):
        vec_time.append(msg.header.stamp)
        vec_pose.append([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z, 
            msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])

    # write images
    last_time = vec_time[0].to_sec()
    cur_time = vec_time[0].to_sec()
    isFirstFrame = True
    bridge_rgb = CvBridge()
    for i, pose in enumerate(vec_pose):
        if not isFirstFrame and ((cur_time - last_time) < 0.03333): # TODO: 30Hz image
            isFirstFrame = False
            # last_time = cur_time
            cur_time = vec_time[i].to_sec()
        else:
            isFirstFrame = True
            last_time = cur_time
            cur_time = vec_time[i].to_sec()
            vehicle_pose = airsim.Pose(airsim.Vector3r(pose[0], pose[1], pose[2]), airsim.Quaternionr(pose[3], pose[4], pose[5], pose[6]))
            client.simSetVehiclePose(vehicle_pose, True) 
            camera_pose = airsim.Pose(airsim.Vector3r(0, 0, 0), airsim.to_quaternion(math.radians(-80), 0, 0)) # TODO: Give an extra rotation to camera
            client.simSetCameraPose("0", camera_pose)
            time.sleep(0.3) # sleep 0.3 sec for engine prepare
            responses = client.simGetImages([
                # camera_name, image_type, pixels_as_float=False, compress=True
                airsim.ImageRequest("0", airsim.ImageType.Scene), 
                airsim.ImageRequest("0", airsim.ImageType.DepthPerspective, True)])
            # responses = client.simGetImages([
            #     airsim.ImageRequest("0", airsim.ImageType.Scene, False, False),
            #     airsim.ImageRequest("1", airsim.ImageType.Scene, False, False),
            #     airsim.ImageRequest("2", airsim.ImageType.Scene, False, False),
            #     airsim.ImageRequest("3", airsim.ImageType.Scene, False, False),
            #     airsim.ImageRequest("4", airsim.ImageType.Scene, False, False),
            #     airsim.ImageRequest("0", airsim.ImageType.DepthPerspective, True, False)])
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

finally:
    bag.close()
