import rospy
from sensor_msgs.msg import Image

def callback(img): 
  print('kk')

if __name__ == '__main__':
  rospy.init_node("kkk", anonymous=True)
  rospy.Subscriber("/hawk/camera_0/image_raw", Image, callback)
  rospy.spin()

