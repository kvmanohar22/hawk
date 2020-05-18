import os
import argparse
import numpy as np
from numpy import genfromtxt
import matplotlib.pyplot as plt
plt.style.use('ggplot')

def analyze(path_1kf, path_3kf):
    data_1kf = genfromtxt(path_1kf, delimiter=',')
    data_3kf = genfromtxt(path_3kf, delimiter=',')
    data_1kf = data_1kf[1:, ...]
    data_3kf = data_3kf[1:, ...]
    
    fig = plt.figure()
    ax = fig.add_subplot(111, xlabel='KeyFrame ID', ylabel='#observations')
    
    plot(ax, data_1kf, ['r.', 'g.', 'b.', 'k.'], '1kf_', 2)
    plot(ax, data_3kf, ['r-', 'g-', 'b-', 'k-'], '3kf_', 4)
    
    plt.legend()
    plt.show()

def plot(ax, data, c, label, w):
  X = data[:, 0]
  ax.plot(X, data[:, 2], c[0], label=label+'obs=2', linewidth=w)
  ax.plot(X, data[:, 3], c[1], label=label+'obs=3', linewidth=w)
  ax.plot(X, data[:, 4], c[2], label=label+'obs=4', linewidth=w)
  ax.plot(X, data[:, 5], c[3], label=label+'obs=4+', linewidth=w)

if __name__ == '__main__':
  parser = argparse.ArgumentParser(description='Analyse number of observations for a smart factor')
  parser.add_argument('file_path_1kf', help='path to the results file')
  parser.add_argument('file_path_3kf', help='path to the results file')
  args = parser.parse_args()
  analyze(args.file_path_1kf, args.file_path_3kf)
