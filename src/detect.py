#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

display = True
disp_size = (1280,720)
out = cv2.VideoWriter('output.mp4',cv2.VideoWriter_fourcc(*'mp4v'), 30, disp_size)

class detector:
  def __init__(self):
    self.image_sub = rospy.Subscriber('/drone1/image_raw',Image,self.callback)
    self.camera_pub= rospy.Publisher('/camera_coord', Float64MultiArray, queue_size=10)
    self.bridge = CvBridge()
    self.rate = rospy.Rate(30)
    self.image = Image()
    self.array = [0,0,0,0]

  def callback(self,data):
    self.image = data
    cv_image = self.bridge.imgmsg_to_cv2(self.image, "bgr8")    
    cv_copy = cv_image.copy()
    frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # define range of black color in HSV
    lower = np.array([0,0,0])
    upper = np.array([0,0,25])

    # Threshold the HSV image to get only black colors
    mask_detection = cv2.inRange(frame, lower, upper)
    cnts = cv2.findContours(mask_detection.copy(),
                            cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)[-2]
    
    if len(cnts)>0:
        blue_area = max(cnts, key=cv2.contourArea)
        (xg,yg,wg,hg) = cv2.boundingRect(blue_area)
        cv_copy = cv2.rectangle(cv_copy,(xg,yg),(xg+wg, yg+hg),(0,255,0),2)
        self.array = [xg+wg/2, yg+hg/2, wg, hg]       # Position of Bbox

    cv_copy = cv2.rectangle(cv_copy,(320,180),(960, 540),(255,0,0),2)
    cv_copy = cv2.line(cv_copy, (640,0), (640,720), (255,255,255), 1)
    cv_copy = cv2.line(cv_copy, (0,360), (1280,360), (255,255,255), 1)

    if display: 
      cv2.imshow('Image window',cv_copy)
      cv2.waitKey(int(1000/30))  

    rospy.loginfo(self.array)
    out.write(cv_copy)

def main():
  try:
    
    rospy.init_node('detector', anonymous=True)
    vis = detector()

    while(not rospy.is_shutdown()):
      vis.camera_pub.publish(Float64MultiArray(data = vis.array))
      vis.rate.sleep()
        
    else:
      print("Shutting down")
      out.release()
      cv2.destroyAllWindows()
      
  except KeyboardInterrupt:
    pass
  
  out.release()
  cv2.destroyAllWindows()

if __name__ == '__main__':
  main()
