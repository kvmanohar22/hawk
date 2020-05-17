import os
import argparse
import numpy as np
from numpy import genfromtxt
import matplotlib.pyplot as plt
plt.style.use('ggplot')

def analyze(path):
    data = genfromtxt(path, delimiter=',')
    data = data[1:, ...]
    plot(data)

def plot(data):
  X = data[:, 0]
  fig = plt.figure()
  ax = fig.add_subplot(111, xlabel='KeyFrame ID', ylabel='#observations')
  ax.plot(X, data[:, 2], 'b-', label='obs=2')
  ax.plot(X, data[:, 3], 'g-', label='obs=3')
  ax.plot(X, data[:, 4], 'r-', label='obs=4')
  ax.plot(X, data[:, 5], 'k-', label='obs=4+')
  plt.legend()
  plt.show()

if __name__ == '__main__':
  parser = argparse.ArgumentParser(description='Analyse number of observations for a smart factor')
  parser.add_argument('file_path', help='path to the results file')
  args = parser.parse_args()
  analyze(args.file_path)
