#!/usr/bin/env python
import rospy
import numpy as np
import geometry_msgs.msg
import mavros_msgs.msg
from math import sin,cos,isnan
# from laser_to_point import scan_cb
# from sensor_msgs.msg import LaserScan
import tf
import roslib
import time
roslib.load_manifest("motion_planning")

X=0
Y=0
Z=0 
R=0 
P=0
YO=0
W=1

class motion:
    def __init__(self,time):
        self.cmd = geometry_msgs.msg.PoseStamped()
        self.time=time

    def default_hold(self):
        self.cmd.pose.position.x=0
        self.cmd.pose.position.y=0
        self.cmd.pose.position.z=3
        self.cmd.pose.orientation.x=0
        self.cmd.pose.orientation.y=0
        self.cmd.pose.orientation.z=0
        self.cmd.pose.orientation.w=0
        return self.cmd

    def circular_path(self,x_curve,y_curve,z_height):
        self.cmd.pose.position.x=x_curve
        self.cmd.pose.position.y=y_curve
        self.cmd.pose.position.z=z_height
        self.cmd.pose.orientation.x=0
        self.cmd.pose.orientation.y=0
        self.cmd.pose.orientation.z=0
        self.cmd.pose.orientation.w=0
        return self.cmd

    def set_goal(self,a,b,c,d,e,f,g):
        self.cmd.pose.position.x=a
        self.cmd.pose.position.y=b
        self.cmd.pose.position.z=c
        self.cmd.pose.orientation.x=d
        self.cmd.pose.orientation.y=e
        self.cmd.pose.orientation.z=f
        self.cmd.pose.orientation.w=g

    def get_goal(self):
        return self.cmd

    def printx(self):
        print "Sent x:",self.cmd.pose.position.x," y:",self.cmd.pose.position.y," z:", self.cmd.pose.position.z

    # def laser_output(self,msg):
    #     self.pc2_msg,self.point_gen=scan_cb(msg)
    #     sum = 0.0
    #     num = 0
    #     for point in self.point_gen:
    #         if not isnan(point[1]) and not isnan(point[0]):
    #             print point[0],point[1]
        
    #     print "Set Over"    
    #     # we can calculate the average y value for example
    #     # print str(sum/num)

    def dist(self,p1,p2):
        return math.sqrt((p1[0]-p2[0])^2+(p1[1]-p2[1])^2)

    def subtract(self,pt,x1,y1):
        x,y=pt[0],pt[1]
        return (x-x1,y-y1)

    def is_in_path(self,pt,source,destination):
        x,y=pt[0],pt[1]
        
        x=x+self.x_pos
        y=y+self.y_pos

        x1=source[0]
        y1=source[1]
        x2=destination[0]
        y2=destination[1]
        m=(y2-y1)/(x2-x1)
        
        d=math.abs((y-y1)-m*(x-x1))/math.sqrt(1+m^2)
        if d<0.2 and dist(pt,source)<=dist(destination,source):
            return True
        else:
            return False

    def trajectory_correct(self,point,source,destination):
        
        if is_in_path(subtract(point,0.2,0.2),source,subtractdestination):
            trajectory_correct((x-0.2,y-0.2))
        else:
            return (x-0.2,y-0.2)
            
    def find_path(self,source,destination,i,total_time):
        global X,Y
        self.x_pos,self.y_pos=X,Y

        self.waypoints=[source]

        for point in self.point_gen:
            if not isnan(point[1]) and not isnan(point[0]):
                A=is_in_path(point,source,destination)
                if A==True:
                    trajectory_correct(point,source,destination)
                    break
                else:
                    pass

        self.waypoints.append(destination)
        return self.waypoints


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

    k=motion(0)

    t = tf.TransformBroadcaster()
    # pub = rospy.Publisher('/mavros/setpoint_position/local', geometry_msgs.msg.PoseStamped, queue_size=1)
    pub2=rospy.Publisher('/mavros/setpoint_attitude/cmd_vel',geometry_msgs.msg.TwistStamped,queue_size=1)
    pub3=rospy.Publisher('/mavros/setpoint_attitude/thrust',mavros_msgs.msg.Thrust,queue_size=1)       

    rospy.Subscriber("/mavros/local_position/pose", geometry_msgs.msg.PoseStamped, callback)
    # rospy.Subscriber("/iris/laser/scan",LaserScan, k.laser_output, queue_size=1)    
    rate = rospy.Rate(100.0)

    i=0.0
    goal_reach=True
    Now=time.time()
    while not rospy.is_shutdown():
        t.sendTransform((X,Y,Z),(R,P,YO,W),rospy.Time.now(),"base_link","map")
        
        # cmd = geometry_msgs.msg.PoseStamped()
        
        
        # if goal_reach==False:
        # cmd=k.circular_path(10+0*3*sin(i/100),0*3*cos(i/100),5+0*sin(i/100))

        
        # k.printx()

        # a=cmd.pose.position.x=0*5*math.sin(i/100)
        # b=cmd.pose.position.y=0*5*math.cos(i/100)
        # c=cmd.pose.position.z=1
        # cmd.pose.orientation.x=0
        # cmd.pose.orientation.y=0
        # cmd.pose.orientation.z=0
        # cmd.pose.orientation.w=0

        # print("Sent x:",a," y:",b," z:",c," i:",i)
        # pub.publish(cmd)
        cmd2 = geometry_msgs.msg.TwistStamped()
        cmd3 = mavros_msgs.msg.Thrust()
        cmd2.twist.angular.x=0.1
        cmd2.twist.angular.y=0.1
        cmd2.twist.angular.z=0.5
        cmd3.thrust=0.35
        pub2.publish(cmd2)
        pub3.publish(cmd3)
        rate.sleep()
        i=i+1
        # print(time.time()-Now)

if __name__ == '__main__':
        try:
            talker()
        except rospy.ROSInterruptException:
            print "Runtime Error"
            pass
