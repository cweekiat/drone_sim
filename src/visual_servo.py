#!/usr/bin/env python

import rospy
from control import drone
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped
import time
from simple_pid import PID

observer_udp = '127.0.0.1:14561' 
display_size = (1280,720)
PID_switch = True

max_lin_vel = 500        # max forward/vertical linear velocity 
max_ang_vel = 500               # max yaw rate 
scale = 1000
tolerance_x = 40
tolerance_y = 180
area_min = 100                                    # Min 1,000 Pixels
area_max = (display_size[0]/4)*(display_size[1]/4) # Max 14,400 Pixels
tolerance_area = [3000,8000]
desired_area = 80*50
search = False  


class visual_servo(drone):

  def __init__(self, name, Kp =1.0, Ki= 0.0, Kd=0.0, sample_time = 0.01):

    self.image_sub = rospy.Subscriber('/camera_coord', Float64MultiArray, self.callback)
    self.vel_pub= rospy.Publisher('/observer/cmd_vel', Float64MultiArray, queue_size=10)
    # self.nmpc_vel_sub = rospy.Subscriber('/nmpc/cmd_vel', TwistStamped, self.MPC_vel)
    
    self.name = name
    self.pidx = PID(1.0, 0.0, 0.0, setpoint = 0)
    self.pidy = PID(1.0, 0.0, 0.0, setpoint = 0)
    self.pida = PID(1.0, 0.0, 0.0, setpoint = desired_area)  

    self.pidx.sample_time = sample_time
    self.pidy.sample_time = sample_time
    self.pida.sample_time = sample_time

    self.pidx.output_limits = (-max_ang_vel, max_ang_vel)
    self.pidy.output_limits = (-max_lin_vel, max_lin_vel)
    self.pida.output_limits = (-max_lin_vel, max_lin_vel)
    
    self.x,self.y,self.w,self.h, self.area = 0,0,0,0,0
    self.v_x,self.v_y,self.yaw_rate = 0,0,0


  def callback(self, data):
    data = data.data
    self.frame_x, self.frame_y, self.frame_w, self.frame_h = data[0], data[1], data[2], data[3]
    self.area = self.w*self.h

def main():
  try:
    rospy.init_node('visual_servo', anonymous=True)
    observer  = visual_servo('observer')
    rate = rospy.Rate(30)
    observer.connectMyCopter(observer_udp) 
    observer.arm_takeoff(10) 
    time.sleep(10)

    while not rospy.is_shutdown():
      if PID_switch == True:
        error_x = observer.x - display_size[0]/2 
        error_y = observer.y - display_size[1]/2
        error_area = observer.area - desired_area
            
        if observer.h == 0 and observer.x == 0:    # No Detection of Drone
          rospy.loginfo('No drone detected')
          if search:
            # Rotate ccw to search for target
            observer.v_x, observer.v_y, observer.yaw_rate = 0,0,300
          else:
            # Stop Moving
            observer.v_x, observer.v_y, observer.yaw_rate = 0,0,0
                
        else:
          if (abs(error_x) > tolerance_x):
            observer.yaw_rate = observer.pidx(error_x) 
          else:
            observer.yaw_rate = 0
          
          if observer.area > area_max:
            observer.v_x = -max_lin_vel
          elif observer.area < area_min and observer.area > 0:
            observer.v_x = max_lin_vel
          else:
            if observer.area > tolerance_area[0] and observer.area < tolerance_area[1]:
              observer.v_x = 0
            else:
              observer.v_x = observer.pida(error_area)
        
        cmd_vel = (observer.v_x/scale, observer.v_y/scale, observer.yaw_rate/scale)
        rospy.loginfo(cmd_vel)
        observer.vel_pub.publish(Float64MultiArray(data = cmd_vel))

    print("Shutting down")
    cmd_vel = (0, 0, 0)
    observer.vel_pub.publish(Float64MultiArray(data = cmd_vel))
          
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
  main()