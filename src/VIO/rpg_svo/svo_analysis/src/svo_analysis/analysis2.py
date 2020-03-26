import os
import argparse
import numpy as np
import matplotlib.pyplot as plt
plt.style.use('ggplot')

def get_val(target):
  return int(target.split('=')[1])

def parse_single_line(l):
  l = l.strip().split()
  idx = get_val(l[0])
  init = get_val(l[1])
  conv = get_val(l[2])
  div = get_val(l[3])
  return np.array([[idx, init, conv, div]])

def analyze(dir):
  file = os.path.join(dir, 'svo.log4')
  with open(file) as f:
    lines = f.readlines()
    print('#lines = {}'.format(len(lines)))
    data = np.zeros((len(lines), 4))
    for idx, line in enumerate(lines):
      np_line = parse_single_line(line)
      data[idx] = np_line
    data_indices = np.argsort(data, axis=0)
    data = data[data_indices[:, 0], ...]
    plot(data)

def plot(data):
  X = data[:, 0]
  fig = plt.figure()
  ax = fig.add_subplot(111, xlabel='KeyFrame ID', ylabel='#features')
  ax.plot(X, data[:, 1], 'b-', label='#initialized')
  ax.plot(X, data[:, 2], 'g-', label='#converged')
  ax.plot(X, data[:, 3], 'r-', label='#diverged')
  plt.legend()
  plt.show()

if __name__ == '__main__':
  parser = argparse.ArgumentParser(description='Analyse feature tracking')
  parser.add_argument('results_dir', help='folder with the results')
  args = parser.parse_args()
  analyze(args.results_dir)
