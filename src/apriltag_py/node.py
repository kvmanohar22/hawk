import rospy
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError

from dt_apriltags import Detector
import numpy, os, yaml

at_detector = Detector(families='tag16h5',
                       nthreads=1,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)

with open('test_info.yaml', 'r') as stream:
    parameters = yaml.load(stream)


cameraMatrix = numpy.array(parameters['sample_test']['K']).reshape((3,3))
camera_params = (cameraMatrix[0,0], cameraMatrix[1,1], cameraMatrix[0,2], cameraMatrix[1,2])

print(camera_params)

rospy.init_node('apriltag_python', anonymous=True)
bridge = CvBridge()

def show_image(img, win_name):
    cv2.imshow(win_name, img)
    cv2.waitKey(3)

def add_tag(tags, color_img):
    for tag in tags:
        for idx in range(len(tag.corners)):
            cv2.line(color_img, tuple(tag.corners[idx-1, :].astype(int)), tuple(tag.corners[idx, :].astype(int)), (0, 255, 0))

        cv2.putText(color_img, str(tag.tag_id),
                org=(tag.corners[0, 0].astype(int)+10,tag.corners[0, 1].astype(int)+10),
                fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                fontScale=0.8,
                color=(0, 0, 255))

def image_callback(img_msg):
    # print("coming!!")
    # rospy.loginfo(img_msg.header)
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))
    show_image(cv_image, "Org Image")
    tags = at_detector.detect(cv_image, True, camera_params, parameters['sample_test']['tag_size'])
    print(tags)
    print('--------------------------------------')
    color_img = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2RGB)
    add_tag(tags, color_img)
    show_image(color_img, "April Tag")

print("Hey!!!")
sub_image = rospy.Subscriber("/hawk/stereo/right/image_raw", Image, image_callback)
# cv2.namedWindow("Image Window", 1)

while not rospy.is_shutdown():
    rospy.spin()