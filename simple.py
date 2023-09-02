import airsim
import math
import numpy as np
client = airsim.MultirotorClient()

client.enableApiControl(True)
client.armDisarm(True)
client.takeoffAsync().join()

height = -60
client.moveToZAsync(height, 8).join()
client.moveToPositionAsync(200, 0, height, velocity=8).join()
# client.moveToPositionAsync(60, 50, height, velocity=8).join()
# client.moveToPositionAsync(120, 50, height, velocity=8).join()
# client.moveToPositionAsync(120, 0, height, velocity=8).join()
# client.moveToPositionAsync(200, 0, height, velocity=8).join()