#!/usr/bin/env python

import rospy
from control import drone
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped
import time
from simple_pid import PID

observer_udp = '127.0.0.1:14550' 
display_size = (1280,720)
PID_switch = True

max_lin_vel = 5000        # max forward/vertical linear velocity 
max_ang_vel = 5000        # max yaw rate 
scale = 1000
tolerance_x = 64
tolerance_y = 180
area_min = 100                                    # Min 1,000 Pixels
area_max = (display_size[0]/8)*(display_size[1]/8) # Max 14,400 Pixels
tolerance_area = [3000,4200]
desired_area = 100*40
search = False  


class visual_servo(drone):

  def __init__(self, name, sample_time = 0.01):

    self.image_sub = rospy.Subscriber('/camera_coord', Float64MultiArray, self.callback)
    self.vel_pub= rospy.Publisher('/drone1/cmd_vel', Float64MultiArray, queue_size=10)
    
    self.name = name
    self.pidx = PID(3.0, 0.0, 0.0, setpoint = 0)
    self.pidy = PID(1.0, 0.0, 0.0, setpoint = 0)
    self.pida = PID(5.0, 0.0, 0.0, setpoint = desired_area)  

    self.pidx.sample_time = sample_time
    self.pidy.sample_time = sample_time
    self.pida.sample_time = sample_time

    self.pidx.output_limits = (-max_ang_vel, max_ang_vel)
    self.pidy.output_limits = (-max_lin_vel, max_lin_vel)
    self.pida.output_limits = (-max_lin_vel, max_lin_vel)
    
    self.x,self.y,self.w,self.h, self.area = 0,0,0,0,0
    self.N,self.D,self.yaw = 0,0,0


  def callback(self, data):
    data = data.data
    self.x, self.y, self.w, self.h = data[0], data[1], data[2], data[3]
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
        error_x = observer.x - display_size[0]/2  #E
        error_y = observer.y - display_size[1]/2	#D
        error_area = observer.area - desired_area #N
            
        if observer.h == 0 and observer.x == 0:    # No Detection of Drone
          rospy.loginfo('No drone detected')
          if search:
            # Rotate ccw to search for target
            observer.N, observer.D, observer.yaw = 0,0,300
          else:
            # Stop Moving
            observer.N, observer.D, observer.yaw = 0,0,0
                
        else:
          if (abs(error_x) > tolerance_x):
            observer.yaw = observer.pidx(error_x) 
          else:
            observer.yaw = 0
          
          if observer.area > area_max:
            observer.N = -max_lin_vel
          elif observer.area < area_min and observer.area > 0:
            observer.N = max_lin_vel
          else:
            if observer.area > tolerance_area[0] and observer.area < tolerance_area[1]:
              observer.N = 0
            else:
              observer.N = observer.pida(error_area)
        
        cmd_vel = (observer.yaw/scale, observer.N/scale, observer.D/scale) #yaw,forward,down
        rospy.loginfo(cmd_vel)
        observer.vel_pub.publish(Float64MultiArray(data = cmd_vel))
        observer.send_servo_commands(cmd_vel[0], cmd_vel[1], cmd_vel[2])#yaw,forward,down
        observer.condition_yaw(cmd_vel[0])

    print("Shutting down")
    cmd_vel = (0, 0, 0)
    observer.vel_pub.publish(Float64MultiArray(data = cmd_vel))
    observer.send_servo_commands(cmd_vel[0], cmd_vel[1], cmd_vel[2])
          
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
  main()
