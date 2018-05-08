

#!/usr/bin/env python
from rospy import Time, Duration
from sensor_msgs.msg import PointCloud 
import tf.transformations
import socket
import rospy, math, random
import numpy as np
import sys
from time import sleep
from geometry_msgs.msg import Twist, Pose, Point, Vector3, Quaternion, PoseStamped
# from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped, Vector3Stamped,PointStamped,Quaternion
# import tf2_ros
import sensor_msgs.point_cloud2 as pc2
# import tf2_geometry_msgs
import sensor_msgs
# import std_msgs
from sensor_msgs.msg import PointField
from sensor_msgs import point_cloud2
ground_height=0
datasize=500*4*32
# tfBuffer = tf2_ros.Buffer()
# listener = tf2_ros.TransformListener(tfBuffer)

MESSAGE = "Hello, World!"
UDP_IP = "192.168.1.49"
UDP_PORT = 27015
topic="/kinect2/sd/points"
sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
data=bytearray(4*datasize*4)
index=1
printOnce=False

pointsToSend=np.zeros((datasize*4+1,), dtype=np.float32)
def on_scan(scan):
    global index,pointsToSend,printOnce
    length = len(scan.data)
    # # trans = tfBuffer.lookup_transform("World", 'velodyne', rospy.Time())
    # # print trans
    # # print "hello"
    pclIndex=0
    # print (type(scan.data))
    # pc=pc2.read_points(scan, field_names = ("x", "y", "z","rgb"), skip_nans=True)
    # # Tpoints=[] 
    # for point in pc:
    #     pointsToSend[index]=point[0]
    #     pointsToSend[index+1]=point[1]
    #     pointsToSend[index+2]=point[2]
    #     pointsToSend[index+3]=point[0]
    #     pointsToSend[index+4]=point[1]
    #     pointsToSend[index+5]=point[2]
        # if(index+6<datasize*6): 
            # index=index+6
        # else:
    #         # print "p"
            # print point
    #         pointsToSend[datasize*6]=pclIndex
            # pclIndex=pclIndex+1
    #         data=np.getbuffer(pointsToSend)
    if printOnce:
        print scan.data
        printOnce=False
    i=0
    while (i*datasize+1<length):
        sock.sendto(scan.data[datasize*i:(datasize*(i+1)-1)], (UDP_IP, UDP_PORT))
        i=i+1
        pclIndex=pclIndex+1
        sleep(0.005)
        
            # index=1
        # vt = tf2_geometry_msgs.do_transform_vector3(v, trans)
        # if (vt.vector.z>ground_height): obstacle_points=obstacle_points+1
        # vvt=vt.vector
        # Tpoints.append(vt)

    # pub.publish(pct)
    #  print " x : %f  y: %f  z: %f" %(p[0],p[1],p[2])
    print pclIndex
if __name__ == '__main__':
    rospy.init_node("PCL2UDP", anonymous=True)
    rospy.Subscriber(topic, sensor_msgs.msg.PointCloud2 , on_scan)
    rospy.spin()
