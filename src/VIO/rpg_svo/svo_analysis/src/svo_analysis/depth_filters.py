import os
import argparse
import numpy as np
import matplotlib.pyplot as plt
plt.style.use('ggplot')

def analyse_depth(results_dir):
    data = np.loadtxt(os.path.join(results_dir, "depth_filter.log"))
    frame_ids = data[:, 0]
    tracked_fts = data[:, 1]
    new_filters = data[:, 2]
    plt.plot(frame_ids, tracked_fts, linewidth=2, label="Tracked features")
    plt.plot(frame_ids, new_filters, linewidth=2, label="New depth filters")
    plt.xlabel("KeyFrame ID")
    plt.ylabel("Features")
    plt.legend()
    plt.show()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Analyse depth filters')
    parser.add_argument('results_dir', help='folder with the results')
    args = parser.parse_args()
    analyse_depth(args.results_dir)
