# Import the required modules
import dlib
import cv2
import argparse as ap
#import get_points 
import time
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CVBridge 

def callback(ros_img):
    bridge = CVBridge()
    # Retrieve an image and Display it.
    img = bridge.imgmsg_to_cv2(ros_img, "bgr8")
    cv2.namedWindow("Image", cv2.WINDOW_NORMAL)
    cv2.imshow("Image", img)
    cv2.destroyWindow("Image")
    """
    # Co-ordinates of objects to be tracked 
    # will be stored in a list named `points`
    points = get_points.run(img) 

    if not points:
        print("ERROR: No object to be tracked.")
        exit()
    
    cv2.namedWindow("Image", cv2.WINDOW_NORMAL)
    cv2.imshow("Image", img)

    # Initial co-ordinates of the object to be tracked 
    # Create the tracker object
    tracker = dlib.correlation_tracker()
    # Provide the tracker the initial position of the object
    tracker.start_track(img, dlib.rectangle(*points[0]))

    while True:
        # Read frame from device or file
        start_time = time.time()
        img = cv_image
        
        # Update the tracker  
        tracker.update(img)
        # Get the position of the object, draw a 
        # bounding box around it and display it.
        rect = tracker.get_position()
        pt1 = (int(rect.left()), int(rect.top()))
        pt2 = (int(rect.right()), int(rect.bottom()))
        cv2.rectangle(img, pt1, pt2, (255, 255, 255), 3)
        # new_img = img[pt1[0] : pt2[0], pt1[1] : pt2[1]]
        # features_dict,features = get_features(img)
        # img = put_features(img,features)
        # print ("Object tracked at [{}, {}] \r".format(pt1, pt2),)
        if dispLoc:
            loc = (int(rect.left()), int(rect.top()-20))
            txt = "Object tracked at [{}, {}]".format(pt1, pt2)
            cv2.putText(img, txt, loc , cv2.FONT_HERSHEY_SIMPLEX, .5, (255,255,255), 1)
        elapsed_time = time.time() - start_time
        print("$$$")
        print(elapsed_time)
        cv2.namedWindow("Image", cv2.WINDOW_NORMAL)
        cv2.imshow("Image", img)
        # Continue until the user presses ESC key
        if cv2.waitKey(1) == 27:
            break

    """
if __name__ == "__main__":
    # Parse command line arguments
    """  
    parser = ap.ArgumentParser()
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('-d', "--deviceID", help="Device ID")
    group.add_argument('-v', "--videoFile", help="Path to Video File")
    parser.add_argument('-l', "--dispLoc", dest="dispLoc", action="store_true")
    args = vars(parser.parse_args())
    """
    rospy.init_node("tracker_node", anonymous = True)
    rospy.Subscriber("/hawk/camera_0/image_raw", Image, callback)
    rospy.spin()
