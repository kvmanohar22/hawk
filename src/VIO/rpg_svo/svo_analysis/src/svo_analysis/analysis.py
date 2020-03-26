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
  n_points_core_kfs = get_val(l[2])
  n_candidate_points = get_val(l[3])
  n_tracked = get_val(l[5])
  after_pose_opt = get_val(l[6])
  FR = get_val(l[-1])
  return np.array([[idx, n_points_core_kfs, n_candidate_points, n_tracked, after_pose_opt, FR]])

def analyze(dir):
  file = os.path.join(dir, 'svo.log2')
  with open(file) as f:
    lines = f.readlines()
    print('#lines = {}'.format(len(lines)))
    data = np.zeros((len(lines), 6))
    for idx, line in enumerate(lines):
      np_line = parse_single_line(line)
      data[idx] = np_line
      if np_line[0, 5] == 2:
        data = data[:idx+1, ...]
        break
    plot(data)

def plot(data):
  X = data[:, 0]
  fig = plt.figure()
  ax = fig.add_subplot(111, xlabel='Frame ID', ylabel='#features')
  # ax.plot(X, data[:, 1], 'r-', label='#features from core keyframes')
  # ax.plot(X, data[:, 2], 'g-', label='#features from candidate list')
  ax.plot(X, data[:, 1]+data[:, 2], 'm-', label='#features reprojected in step B')
  ax.plot(X, data[:, 3], 'b-', label='#tracked features after step B')
  ax.plot(X, data[:, 4], 'k-', label='#tracked features after step C')
  plt.legend()
  plt.show()

if __name__ == '__main__':
  parser = argparse.ArgumentParser(description='Analyse feature tracking')
  parser.add_argument('results_dir', help='folder with the results')
  args = parser.parse_args()
  analyze(args.results_dir)
