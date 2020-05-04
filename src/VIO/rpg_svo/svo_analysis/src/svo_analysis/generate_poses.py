import numpy as np
from collections import OrderedDict 

def parse(path):
    f = open(path)
    lines = f.readlines()
    lines = [line.strip() for line in lines]
    
    kf_pose = OrderedDict()
    for line in lines:
        idx = line.split(' ')[0]
        pose = line.split(' ', 1)[1] 
        kf_pose[idx] = pose 
    f.close()

    # write to file
    f = open("final.txt", 'w') 
    for (k,v) in kf_pose.iteritems():
        f.write(v+"\n")
    f.close()

if __name__ == '__main__':
    path='trajectory.txt'
    parse(path)

