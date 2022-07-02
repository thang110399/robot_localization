#!/usr/bin/env python3
import roslib; roslib.load_manifest('my_robot_controller')
import math
import rospy
import tf.transformations
from geometry_msgs.msg import Twist
from connection import RS485
from PID import PID

def callback(msg):
    global myData
    rospy.loginfo("Received a /cmd_vel message!")
    rospy.loginfo(msg)
    myData = msg


def listener():
    rospy.init_node('omni_robot')
    pub=rospy.Publisher("/odom/kinematic_raw", Twist, queue_size=10)
    rospy.Subscriber('cmd_vel', Twist, callback)
    rate = rospy.Rate(10)
    global alpha
    global d
    while not rospy.is_shutdown():
        # Then set your wheel speeds (using wheel_left and wheel_right as examples)
        v1_dat = math.sqrt(2)/2*(-myData.linear.x + myData.linear.y) + myData.angular.z*d

        v2_dat = math.sqrt(2)/2*(-myData.linear.x - myData.linear.y) + myData.angular.z*d

        v3_dat = math.sqrt(2)/2*( myData.linear.x - myData.linear.y) + myData.angular.z*d

        v4_dat = math.sqrt(2)/2*( myData.linear.x + myData.linear.y) + myData.angular.z*d
        # Doc van toc v1 (van toc banh 1)
        v1 = rs1.read('hr',40)
        if v1 > 32768:
            v1_thuc = v1 - 65536 
        else: 
            v1_thuc = v1

            # Doc van toc v3 (van toc banh 3)
        v3 = rs1.read('hr',50)
        if v3 > 32768:
            v3_thuc = v3 - 65536 
        else: 
            v3_thuc = v3
        
        # Doc van toc v2 (van toc banh 2)
        v2 = rs2.read('hr',40)
        if v2 > 32768:
            v2_thuc = v2 - 65536 
        else: 
            v2_thuc = v2

        # Doc van toc v4 (van toc banh 4)
        v4 = rs2.read('hr',50)
        if v4 > 32768:
            v4_thuc = v4 - 65536 
        else: 
            v4_thuc = v4


        # Ham PID va nap analog dong co 1
        output = pid1.compute(abs(v1_dat), abs(v1_thuc))
        if isinstance(output,float) == True:
            control_1=int(output)
        if v1_dat >= 0:
            rs1.write('reg',1,control_1)
            rs1.write('reg',11,0)
        else:
            rs1.write('reg',1,control_1)
            rs1.write('reg',11,1)

        # Ham PID va nap analog dong co 2
        output = pid2.compute(abs(v2_dat), abs(v2_thuc))
        if isinstance(output,float) == True:
            control_2=int(output)
        if v2_dat >= 0:
            rs2.write('reg',1,control_2)
            rs2.write('reg',11,0)
        else:
            rs2.write('reg',1,control_2)
            rs2.write('reg',11,1)

        
        # Ham PID va nap analog dong co 3
        output = pid3.compute(abs(v3_dat), abs(v3_thuc))
        if isinstance(output,float) == True:
            control_3=int(output)
        if v3_dat >= 0:
            rs1.write('reg',2,control_3)
            rs1.write('reg',12,0)
        else:
            rs1.write('reg',2,control_3)
            rs1.write('reg',12,1)


        # Ham PID va nap analog dong co 4
        output = pid4.compute(abs(v4_dat), abs(v4_thuc))
        if isinstance(output,float) == True:
            control_4=int(output)
        if v4_dat >= 0:
            rs2.write('reg',2,control_4)
            rs2.write('reg',12,0)
        else:
            rs2.write('reg',2,control_4)
            rs2.write('reg',12,1)
        

        V = math.sqrt(2)/4*(- v1_thuc - v2_thuc + v3_thuc + v4_thuc) #van toc x theo truc gan tren xe
        Vn =  math.sqrt(2)/4*(v1_thuc - v2_thuc - v3_thuc + v4_thuc) #van toc y theo truc gan tren xe
        omega = 1/(4*d)*(v1_thuc + v2_thuc + v3_thuc + v4_thuc) # van toc goc
        Vx = math.cos(alpha)*V - math.sin(alpha)*Vn #van toc x theo truc gan tren goc toa do
        Vy = math.sin(alpha)*V + math.cos(alpha)*Vn #van toc y theo truc gan tren goc toa do

        alpha = alpha + omega*0.02 # goc lech giua truc toa do cua moi truong va truc toa do cua xe, 0.02s la thoi gian lay mau 
        twist = Twist()
        twist.linear.x = V
        twist.linear.y = Vn
        twist.linear.z = 0

        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = omega
        pub.publish(twist)
        rate.sleep()

        # # pub =rospy.Publisher("/odom/kinematic_raw", Twist, queue_size=10)
        # pub.publish(twist)
        # twist.linear.x = myData.linear.x
        # twist.linear.y = 0
        # twist.linear.z = 0
        # twist.angular.x = 0
        # twist.angular.y = 0
        # twist.angular.z = myData.angular.z
        # twist = Twist()
        # # pub =rospy.Publisher("/odom/kinematic_raw", Twist, queue_size=10)
        
if __name__ == '__main__':
    rs1 = RS485(port=1)
    rs2 = RS485(port=2)
    myData = Twist()
    alpha = 0 # goc lech giua truc toa do cua moi truong va truc toa do cua xe 
    d = 21 # 21 cm
      # Cai dat PID cho dong co 1
    pid1 = PID(kp=4, ki=0.1, kd=0.0, direction=1)
    pid1.addOutputOffset(0)
    pid1.updateTime = 100
    pid1.setOutputLimits(0, 2000)

    # Cai dat PID cho dong co 2
    pid2 = PID(kp=3.8, ki=0.13, kd=0.0, direction=1)
    pid2.addOutputOffset(0)
    pid2.updateTime = 100
    pid2.setOutputLimits(0, 2000)    

    # Cai dat PID cho dong co 3
    pid3 = PID(kp=3.6, ki=0, kd=0.0, direction=1)
    pid3.addOutputOffset(0)
    pid3.updateTime = 100
    pid3.setOutputLimits(0, 2000)

    # Cai dat PID cho dong co 4
    pid4 = PID(kp=3.8, ki=0.11, kd=0.0, direction=1)
    pid4.addOutputOffset(0)
    pid4.updateTime = 100
    pid4.setOutputLimits(0, 2000)
    listener()
    
    