import sensor_msgs.point_cloud2 as pc2
import rospy
from sensor_msgs.msg import PointCloud2, LaserScan
import laser_geometry.laser_geometry as lg
import math

# rospy.init_node("laserscan_to_pointcloud")


# pc_pub = rospy.Publisher("converted_pc", PointCloud2, queue_size=1)

def scan_cb(msg):
    lp = lg.LaserProjection()

    # convert the message of type LaserScan to a PointCloud2
    pc2_msg = lp.projectLaser(msg)

    # now we can do something with the PointCloud2 for example:
    # publish it
    # pc_pub.publish(pc2_msg)
    
    # convert it to a generator of the individual points
    point_generator = pc2.read_points(pc2_msg)
    

    # we can access a generator in a loop
    

    # or a list of the individual points which is less efficient
    # point_list = pc2.read_points_list(pc2_msg)

    # we can access the point list with an index, each element is a namedtuple
    # we can access the elements by name, the generator does not yield namedtuples!
    # if we convert it to a list and back this possibility is lost
    # print(point_list[len(point_list)/2].x)
    return pc2_msg,point_generator



# rospy.Subscriber("/scan", LaserScan, scan_cb, queue_size=1)
# rospy.spin()