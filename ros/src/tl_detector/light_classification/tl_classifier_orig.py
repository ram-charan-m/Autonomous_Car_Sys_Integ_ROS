from styx_msgs.msg import TrafficLight
import cv2
import numpy as np

class TLClassifier(object):
    def __init__(self):
        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image
        Args:
            image (cv::Mat): image containing the traffic light
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        result = TrafficLight.UNKNOWN
        # creating red color levels 
        red = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        red_low = np.array([0,50,50])
        red_high = np.array([10,255,255])
        
        red1 = cv2.inRange(red, red_low , red_high)

        red_low = np.array([170,50,50])
        red_high = np.array([180,255,255])
        red2 = cv2.inRange(red, red_low , red_high)

        converted_img = cv2.addWeighted(red1, 1.0, red2, 1.0, 0.0)

        blur_img = cv2.GaussianBlur(converted_img,(15,15),0)


        #edges = cv2.Canny(imgray,thresh,thresh*3)

        circles = cv2.HoughCircles(blur_img,cv2.HOUGH_GRADIENT,0.5,41, param1=70,param2=30,minRadius=5,maxRadius=150)

        found = False 
        if circles is not None:
            result = TrafficLight.RED
        #    for i in circles[0,:3]:
        #        cv2.circle(output,(i[0],i[1]),maxRadius,(255, 100, 100),2)
      
        
        #need to include more image, so ignore other colors
        #green may be trees.  Just look for red lights
        #if red_area > 40:
        return result
