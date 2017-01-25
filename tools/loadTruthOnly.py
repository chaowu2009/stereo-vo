"""Example of pykitti.odometry usage."""
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

import pykitti

__author__ = "Lee Clement"
__email__ = "lee.clement@robotics.utias.utoronto.ca"

# Change this to the directory where you store KITTI data
basedir = '/home/cwu/Downloads/dataset'

# Specify the dataset to load
sequence = '00'

# Optionally, specify the frame range to load
frame_range = range(0, 20, 5)

# Load the data
# dataset = pykitti.odometry(basedir, sequence)
dataset = pykitti.odometry(basedir, sequence, frame_range)

# Load some data

dataset.load_poses()        # Ground truth poses are loaded as 4x4 arrays


# Display some of the data
np.set_printoptions(precision=4, suppress=True)

#print('\nFirst timestamp: ' + str(dataset.timestamps[0]))
print('\nSecond ground truth pose:\n' + str(dataset.T_w_cam0[1]))
print(dataset)

plt.show()

