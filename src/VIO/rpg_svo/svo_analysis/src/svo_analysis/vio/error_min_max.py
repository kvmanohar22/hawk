import os
import argparse
import numpy as np
from numpy import genfromtxt
import matplotlib.pyplot as plt
plt.style.use('ggplot')

# 26 -> max
# 27 -> median

def analyze(path_1kf, path_3kf):
    data_1kf = genfromtxt(path_1kf, delimiter=',')
    data_3kf = genfromtxt(path_3kf, delimiter=',')
    data_1kf = data_1kf[1:, ...]
    data_3kf = data_3kf[1:, ...]

    error1_max = data_1kf[:, 26]
    error1_median = data_1kf[:, 27]
    valid_idxs1 = error1_median > 0
    error1_median = error1_median[valid_idxs1]
    error1_max = error1_max[valid_idxs1]

    error2_max = data_3kf[:, 26]
    error2_median = data_3kf[:, 27]
    valid_idxs2 = error2_median > 0
    error2_median = error2_median[valid_idxs2]
    error2_max = error2_max[valid_idxs2]


    fig = plt.figure()
    ax = fig.add_subplot(111, xlabel='KeyFrame ID', ylabel='error median')

    # max
    # ax.plot(error1_max, label='1.0 3d error median', linewidth=2)
    ax.plot(error2_max, label='1e-12 error median', linewidth=2)

    # median
    # ax.plot(error1_median, label='1.0 3d error median', linewidth=2)
    # ax.plot(error2_median, label='1e-12 error median', linewidth=2)


    plt.legend()
    plt.show()

if __name__ == '__main__':
  parser = argparse.ArgumentParser(description='Analyse number of observations for a smart factor')
  parser.add_argument('file_path_1kf', help='path to the results file')
  parser.add_argument('file_path_3kf', help='path to the results file')
  args = parser.parse_args()
  analyze(args.file_path_1kf, args.file_path_3kf)

