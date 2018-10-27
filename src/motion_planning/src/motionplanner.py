#!/usr/bin/env python
import rospy
import numpy as np
import geometry_msgs.msg
import math
import tf
import roslib
roslib.load_manifest("motion_planning")

X=0
Y=0
Z=0 
R=0 
P=0
YO=0
W=1


def callback(K):
    global X,Y,Z,R,P,YO,W
    X=K.pose.position.x
    Y=K.pose.position.y
    Z=K.pose.position.z
    R=K.pose.orientation.x
    P=K.pose.orientation.y
    YO=K.pose.orientation.z
    W=K.pose.orientation.w

def talker():
    rospy.init_node('motionplan', anonymous=True)

    t = tf.TransformBroadcaster()
    pub = rospy.Publisher('/mavros/setpoint_position/local', geometry_msgs.msg.PoseStamped, queue_size=1)
    rospy.Subscriber("/mavros/local_position/pose", geometry_msgs.msg.PoseStamped, callback)

    rate = rospy.Rate(10.0)

    i=0.0
    while not rospy.is_shutdown():
        t.sendTransform((X,Y,Z),(R,P,YO,W),rospy.Time.now(),"base_link","map")

        cmd = geometry_msgs.msg.PoseStamped()
        a=cmd.pose.position.x=0*5*math.sin(i/100)
        b=cmd.pose.position.y=0*5*math.cos(i/100)
        c=cmd.pose.position.z=1
        cmd.pose.orientation.x=0
        cmd.pose.orientation.y=0
        cmd.pose.orientation.z=0
        cmd.pose.orientation.w=0

        print("Sent x:",a," y:",b," z:",c," i:",i)
        pub.publish(cmd)
        rate.sleep()
        i=i+1

if __name__ == '__main__':
        try:
                talker()
        except rospy.ROSInterruptException:
                print "Runtime Error"
                pass
