#!/usr/bin/env python

"""
ON THE RASPI: roslaunch raspicam_node camerav2_320x240.launch enable_raw:=true

   0------------------> x (cols) Image Frame
   |
   |        c    Camera frame
   |         o---> x
   |         |
   |         V y
   |
   V y (rows)


SUBSCRIBES TO:
    /raspicam_node/image: Source image topic
    
PUBLISHES TO:
    /blob/image_blob : image with detected blob and search window
    /blob/image_mask : masking    
    /blob/point_blob : blob position in adimensional values wrt. camera frame

"""

import rospy
import cv2 as cv
import math
import time

from std_msgs.msg           import String
from sensor_msgs.msg        import Image
from geometry_msgs.msg      import Point
from cv_bridge              import CvBridge, CvBridgeError
import numpy as np

class LaneDetector:

    def __init__(self, name):

        self.name = name
        self.bridge = CvBridge()
        self.result_pub = rospy.Publisher("/racer/lane_detected",Image,queue_size=1)
        self.image_sub = rospy.Subscriber("/racer/camera1/image_raw",Image,self.callback)
        self.minLineLength = 50
        self.maxLineGap = 100
        self.font = cv.FONT_HERSHEY_SIMPLEX
        self.fontScale = 1
        self.thickness = 5 

    def regOfInt(self,image):
        roi_size = np.array([[0,435], [0,600], [1000,600], [1000,435]])
        mask = np.zeros_like(image)
        cv.fillConvexPoly(mask, roi_size, (255,255,255))
        masked = cv.bitwise_and(image, mask)
        return masked
    
    def mean(self,list):
        return float(sum(list)) / max(len(list), 1)

    def callback(self,data):
        #--- Assuming image is 320x240
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        (rows,cols,channels) = cv_image.shape
        frame = cv_image
        kernel = np.ones((5,5),np.uint8)

        gray = cv.cvtColor(frame, cv.COLOR_RGB2GRAY)
        hsv = cv.cvtColor(frame, cv.COLOR_RGB2HSV)
        frm_cpy = np.copy(frame)
        
        img_thd_mark = cv.inRange(hsv, (0, 0, 212), (0, 255, 255))
        img_thd_lane = cv.inRange(hsv, (0, 0, 82), (204, 255, 255))

        img_thd = cv.bitwise_xor(img_thd_lane, img_thd_mark)
        img_thd = cv.erode(img_thd,kernel,iterations = 1)

        blur = cv.GaussianBlur(img_thd, (5,5), 0)
        edge = cv.Canny(blur, 50,150)
        roi = self.regOfInt(edge)
        blur_roi = cv.GaussianBlur(roi, (5,5), 0)
        dilation = cv.dilate(blur_roi,kernel,iterations = 1)
        lines = cv.HoughLinesP(dilation,1,np.pi/180,100,self.minLineLength,self.maxLineGap)
        line_img = self.dispLine(frame, lines)

        # cv.imshow("result", line_img)
        try:
            self.result_pub.publish(self.bridge.cv2_to_imgmsg(line_img, "bgr8"))
        except CvBridgeError as e:
            print(e)   

    def dispLine(self,image, lines):

        bLeftValues     = []  # b of left lines
        bRightValues    = []  # b of Right lines
        mPositiveValues = []  # m of Left lines
        mNegitiveValues = []  # m of Right lines

        if lines is not None:
            for line in lines:         
                x1,y1,x2,y2 = line.reshape(4)
                m = float((y2-y1))/(x2-x1)
                b = y1 - x1*m

                if(m < 0):
                    mPositiveValues.append(m)
                    bLeftValues.append(b)
                elif(m > 0):
                    mNegitiveValues.append(m)
                    bRightValues.append(b)

            imshape = image.shape
            y_max   = 450 
            y_min   = 435           
            
            AvgPositiveM = self.mean(mPositiveValues)
            AvgNegitiveM = self.mean(mNegitiveValues)
            AvgLeftB     = self.mean(bLeftValues)
            AvgRightB    = self.mean(bRightValues)

            if AvgPositiveM != 0:
                x1_Left = (y_max - AvgLeftB)/AvgPositiveM
                y1_Left = y_max
                x2_Left = (y_min - AvgLeftB)/AvgPositiveM
                y2_Left = y_min
                # cv.line(image, (int(x1_Left), int(y1_Left)), (int(x2_Left), int(y2_Left)), (0,0,255), 5) #avg Left Line
                X_Left = x2_Left + (x1_Left - x2_Left)/2
                Y_Left = y2_Left + (y1_Left - y2_Left)/2
                image = cv.putText(image, "|", (int(X_Left), int(Y_Left)), self.font,self.fontScale, (0,0,255), self.thickness, cv.LINE_AA)

            if AvgNegitiveM != 0:
                x1_Right = (y_max - AvgRightB)/AvgNegitiveM
                y1_Right = y_max
                x2_Right = (y_min - AvgRightB)/AvgNegitiveM
                y2_Right = y_min
                # cv.line(image, (int(x1_Right), int(y1_Right)), (int(x2_Right), int(y2_Right)), (255,0,0), 5) #avg Right Line
                X_Right = x2_Right + (x1_Right - x2_Right)/2
                Y_Right = y2_Right + (y1_Right - y2_Right)/2 
                image = cv.putText(image, "|", (int(X_Right), int(Y_Right)), self.font,self.fontScale, (255,0,0), self.thickness, cv.LINE_AA)


            # centerR = math.sqrt((x2_Right-x1_Right)**2 + (y2_Right-y1_Right)**2)
            image = cv.putText(image, "|", (int(imshape[0]/2), int(imshape[1]/2)), self.font,self.fontScale, (255,255,255), self.thickness, cv.LINE_AA)
            if AvgPositiveM != 0 and AvgNegitiveM != 0 :
                theta = math.atan((AvgPositiveM - AvgNegitiveM)/ (1 + AvgNegitiveM * AvgPositiveM))
                theta = theta * 180/3.14
                X = X_Left + (X_Right - X_Left)/2
                org_ = (int(X), int(imshape[1]/2))
                
                image = cv.putText(image, "|", org_,self.font,self.fontScale, (0,255,255), self.thickness, cv.LINE_AA)
                image = cv.putText(image, str(int(X-int(imshape[0]/2))), (50,50), self.font,self.fontScale, (0,255,255), 2, cv.LINE_AA)
        return image

        

def main():
    

    ic = LaneDetector("VESC")
    rospy.init_node('lane_detector', anonymous=False)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    
    cv.destroyAllWindows()

if __name__ == '__main__':
    main()
