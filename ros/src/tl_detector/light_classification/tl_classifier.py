from styx_msgs.msg import TrafficLight
import cv2
import numpy as np
import rospy

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        self.state = TrafficLight.UNKNOWN
        

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        # color thresholding technique to find the red colour in the image
        # https://stackoverflow.com/questions/30331944/finding-red-color-in-image-using-python-opencv
        hsv_img = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
        min_thresh = np.array([0, 120, 120],np.uint8)
        max_thresh = np.array([10, 255, 255],np.uint8)
        mask = cv2.inRange(hsv_img, min_thresh, max_thresh)
        red_pix = cv2.countNonZero(mask)
        if red_pix > 50:
            self.state = TrafficLight.RED
        else:
            self.state = TrafficLight.UNKNOWN

        return self.state
