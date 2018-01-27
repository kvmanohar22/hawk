# Import the required modules
import dlib
import cv2
import argparse as ap
import get_points 
import time
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge 

first_call = True
points = None
tracker = dlib.correlation_tracker()
def callback(ros_img):
    global first_call, points, tracker
    bridge = CvBridge()
    # Retrieve an image and Display it.
    img = bridge.imgmsg_to_cv2(ros_img, "bgr8")
    cv2.imshow("Image", img)
    #cv2.waitKey(1)
   # cv2.destroyWindow("Image")
    # Co-ordinates of objects to be tracked 
    # will be stored in a list named `points`
    
    if first_call:
        points = get_points.run(img)
        tracker.start_track(img, dlib.rectangle(*points[0]))
        first_call = False 
        pass
    #if not points:
    #    print("ERROR: No object to be tracked.")
    #    exit()

    #cv2.waitKey(1)
    # Initial co-ordinates of the object to be tracked 
    # Create the tracker object
    # Provide the tracker the initial position of the object
    #tracker.start_track(img, dlib.rectangle(*points[0]))
    # Read frame from device or file
    start_time = time.time()
        
    # Update the tracker  
    tracker.update(img)
    # Get the position of the object, draw a 
    # bounding box around it and display it.
    rect = tracker.get_position()
    pt1 = (int(rect.left()), int(rect.top()))
    pt2 = (int(rect.right()), int(rect.bottom()))
    cv2.rectangle(img, pt1, pt2, (255, 255, 255), 3)
    # print ("Object tracked at [{}, {}] \r".format(pt1, pt2),)
    loc = (int(rect.left()), int(rect.top()-20))
    txt = "Object tracked at [{}, {}]".format(pt1, pt2)
    cv2.putText(img, txt, loc , cv2.FONT_HERSHEY_SIMPLEX, .5, (255,255,255), 1)
    elapsed_time = time.time() - start_time
    print("$$$")
    print(elapsed_time)
    cv2.imshow("Image", img)
    cv2.waitKey(1)
    # Continue until the user presses ESC key
    #if cv2.waitKey(1) == 27:
    #    break

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
    print("Hello")
    cv2.namedWindow("Image", cv2.WINDOW_NORMAL)
    rospy.init_node("tracker_node", anonymous = True)
    rospy.Subscriber("/hawk/camera_0/image_raw", Image, callback)
    rospy.spin()
