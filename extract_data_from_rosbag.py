from fileinput import close
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import argparse
import sys

# rgb_path = './extracted/rgb/'     # absolute path of extracted rgb images
# depth_path = './extracted/depth/' # absolute path of extracted depth images
parser = argparse.ArgumentParser(description='Get images data from rosbag')
parser.add_argument('--bag', metavar='bag', default='./mybag.bag', help='ROS bag file name', required=True)
parser.add_argument('--out_file', metavar='out_file', default='./myGT.txt', help='output GT file name', required=True)

#print help if no argument is specified
if len(sys.argv) < 1:
    parser.print_help()
    sys.exit(0)

#parse the args
parsed = parser.parse_args()

bridge = CvBridge()
# path = "/mnt/g/town/path_data/"
file = open(parsed.out_file,'w')
# path = '/mnt/d/car/'
with rosbag.Bag(parsed.bag, 'r') as bag:
    for topic,msg,t in bag.read_messages():
        # if topic == "/airsim_node/drone_1/depth/DepthPerspective": 
        #     cv_image = bridge.imgmsg_to_cv2(msg, '32FC1')
        #     timestr = "%.8f" %  msg.header.stamp.to_sec()
        #     image_name = timestr + '.png'# an extension is necessary
        #     cv2.imwrite(depth_path + image_name, cv_image)
        #     print(depth_path + image_name)
        # if topic == "/airsim_node/drone_1/raw/Scene": 
        #     cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        #     timestr = "%.8f" %  msg.header.stamp.to_sec()
        #     image_name = timestr + '.jpg'# an extension is necessary
        #     cv2.imwrite(rgb_path + image_name, cv_image)
        #     print(rgb_path + image_name)
        # for i,j,k in os.walk(rgb_path):
        #     for s in k:
                if topic == "/airsim_node/PhysXCar/odom_local_ned":             
                    # if (s[0:-4] == msg.header.stamp.to_sec()):
                        q = []
                        q.append(msg.pose.pose.orientation.x)
                        q.append(msg.pose.pose.orientation.y)
                        q.append(msg.pose.pose.orientation.z)
                        q.append(msg.pose.pose.orientation.w)
                        a11 = 1.0 - 2 * (q[1] * q[1] + q[2] * q[2])
                        a12 = 2 * (q[0] * q[1] - q[3] * q[2])
                        a13 = 2 * (q[3] * q[1] + q[0] * q[2])
                        a14 = msg.pose.pose.position.x
                        a21 = 2 * (q[0] * q[1] + q[3] * q[2])
                        a22 = 1.0 - 2 * (q[0] * q[0] + q[2] * q[2])
                        a23 = 2 * (q[1] * q[2] - q[3] * q[0])
                        a24 = msg.pose.pose.position.y
                        a31 = 2 * (q[0] * q[2] - q[3] * q[1])
                        a32 = 2 * (q[1] * q[2] + q[3] * q[0])
                        a33 = 1.0 - 2 * (q[0] * q[0] + q[1] * q[1])
                        a34 = msg.pose.pose.position.z
                        # line1 = [a11, a12, a13, a14]
                        # line2 = [a21, a22, a23, a24]
                        # line3 = [a31, a32, a33, a34]
                        # line4 = [0, 0, 0, 1]
                        # data = np.array([line1, line2, line3, line4])
                        # np.savetxt(path + str(msg.header.stamp.to_sec()) + ".txt", data)
                        print(msg.header.stamp.to_sec())
                        file.write(str(a11) + ' ' + str(a12) + ' ' + str(a13) + ' ' + str(a14))
                        file.write("\n")
                        # file.write(' ')
                        file.write(str(a21) + ' ' + str(a22) + ' ' + str(a23) + ' ' + str(a24))
                        # file.write(' ')
                        file.write("\n")
                        file.write(str(a31) + ' ' + str(a32) + ' ' + str(a33) + ' ' + str(a34))
                        file.write("\n")
                        file.write("0 0 0 1")
                        file.write("\n")

file = close()
