#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import cv2
import time



def image_callback(img_msg):
    """
    When a new image is published, save the last frame in a global variable
    """
    #print("Received an image!")
    bridge = CvBridge()
    try:
        # Convert from sensor_msgs::Image to cv::Mat
    	cv_image = bridge.imgmsg_to_cv2(img_msg, desired_encoding="passthrough")
    	# Access global variable and store image as numpy.array
    except CvBridgeError as ex:
	    print ex
    else:
        # Save your OpenCV2 image as a jpeg
        cv2.imwrite(time.strftime("%H:%M:%S") + 'camera_image.jpeg', cv_image)

def main():
    """
    Initialize and run the rospy node.
    """
    rospy.init_node("CameraRecorder")
    rospy.loginfo("----- CameraRecorder -----")
    image_topic = "/quadrotor/ardrone/bottom/ardrone/bottom/image_raw"
    # Crate a subscriber for the camera
    rospy.Subscriber( image_topic, Image, image_callback)
    #rate = rospy.Rate(1) # 1 Hz

    #rate.sleep() # Call all the callback at fixed rate
    rospy.spin()

if __name__ == "__main__":
    main()
