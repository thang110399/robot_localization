#!/usr/bin/env python3
import math

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3,TransformStamped

def talk_to_me():
    rospy.init_node('publisher_node')
    rospy.loginfo("Node has been started.")
    msg = Odometry()
    # rs1 = RS485(port=1)
    # rs2 = RS485(port=2)
    pub = rospy.Publisher("/odom/kinematic_raw", Odometry, queue_size=1)
    br = tf.TransformBroadcaster() 
    rate = rospy.Rate(2)
    
    while not rospy.is_shutdown():
        # a = rs1.read('hr', 30)
        # if a > 32768:
        #     v1 = a -65536
        # else:
        #     v1 = a

        # c = rs1.read('hr', 31)
        # if c > 32768:
        #     v3 = c -65536
        # else:
        #     v3 = c

        # b = rs2.read('hr', 30)
        # if b > 32768:
        #    v2 = b -65536
        # else:
        #     v2 = b

        # d = rs2.read('hr', 31)
        # if d > 32768:
        #     v4 = d -65536
        # else:
        #     v4 = d

        # l = 5 # khoang cach tu tam den banh xe
        # alpha = 45 # goc lech giua truc truc gốc và trục xe

        # V = math.sqrt(2)/4*(- v1 - v2 + v3 + v4) #van toc x theo truc gan tren xe
        # Vn =  math.sqrt(2)/4*(v1 - v2 - v3 + v4) #van toc y theo truc gan tren xe
        # omega = 1/(4*l)*(v1 + v2 + v3 + v4) # van toc goc
        # Vx = math.cos(alpha)*V - math.sin(alpha)*Vn #van toc x theo truc gan tren goc toa do
        # Vy = math.sin(alpha)*V + math.cos(alpha)*Vn #van toc y theo truc gan tren goc toa do

        # msg.linear.x = Vx
        # msg.linear.y = Vy
        # msg.linear.z = 0

        # msg.angular.x = 0
        # msg.angular.y = 0
        # msg.angular.z = omega
        quat = tf.transformations.quaternion_from_euler(0,0,0)
        odom_trans = TransformStamped()
        odom_trans.header.stamp = rospy.Time.now()
        odom_trans.header.frame_id = "odom"
        odom_trans.child_frame_id = "base_link"
        odom_trans.transform.translation.x = 0
        odom_trans.transform.translation.y = 0
        odom_trans.transform.translation.z = 0.0
        odom_trans.transform.rotation = quat
        br.sendTransform((0,0,0.0), quat, odom_trans.header.stamp, odom_trans.child_frame_id, odom_trans.header.frame_id)

        msg.header.frame_id = "odom"
        msg.child_frame_id= "base_link"
        msg.header.stamp=rospy.Time.now()
        msg.twist.twist.linear.x = 0
        msg.twist.twist.linear.y = 0
        msg.twist.twist.linear.z = 0
        msg.twist.twist.angular.x = 0
        msg.twist.twist.angular.y = 0
        msg.twist.twist.angular.z = 0
        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        talk_to_me()
    except rospy.ROSInterruptException:
        pass
