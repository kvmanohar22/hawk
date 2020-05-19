import rosbag
import os
import argparse

def write(bag, msgs, offset, reverse=False):
    if reverse:
        msgs.reverse()
    for (t, msg) in msgs:
        bag.write('camera/image_raw', msg, t+offset)

def augment(path):
    bag = rosbag.Bag(path)
    start_time = bag.get_start_time()
    end_time = bag.get_end_time()
    duration = end_time-start_time 
    msgs = [] 
    for topic, msg, t in bag.read_messages(topics=['camera/image_raw']):
        msgs.append([t,msg])
    print("#messages = {}".format(len(msgs)))

    # create a new bag based on the above messages
    new_bag = rosbag.Bag(path+".augment.bag", 'w')
    write(new_bag, msgs, start_time, False)
    write(new_bag, msgs, 1*duration, True)
    write(new_bag, msgs, 2*duration, True)
    write(new_bag, msgs, 3*duration, True)
    new_bag.close()

if __name__ == '__main__':
  parser = argparse.ArgumentParser(description='Augment rosbag')
  parser.add_argument('bag_path', help='path to the results file')
  args = parser.parse_args()
  augment(args.bag_path)
