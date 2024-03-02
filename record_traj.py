# Written by YuantaoChen
import time

import numpy as np
import airsim
import threading


global data_list
data_list = []


class Recording(threading.Thread):
    def run(self):
        write()


class Publishing(threading.Thread):
    def run(self):
        state()


def state():
    client = airsim.MultirotorClient()
    while True:
        state = client.getMultirotorState()
        # timestamp, x, y, z, qx, qy, qz, qw
        state_data = np.concatenate(([state.timestamp], state.kinematics_estimated.position.to_numpy_array(),
                                     state.kinematics_estimated.orientation.to_numpy_array()))
        data_list.append(state_data)


def write():
    while True:
        time.sleep(1)
        data = np.array(data_list)
        np.savetxt('./GT.txt', data)
        print("saving...")


thread1 = Publishing()
thread2 = Recording()
thread1.start()
thread2.start()

thread1.join()




