#!/usr/bin/env python

# Python libs
import sys
import time

# Ros libraries
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16MultiArray, Float32MultiArray, String

VERBOSE = False

class api_subscriber:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        self.msg = Joy()
        self.pub = api_publisher()
        # subscribed Topic
        self.subscriber = rospy.Subscriber("/unity_ros",
                                           String, self.callback,  queue_size=1)
        if VERBOSE:
            print("/unity_ros")

    def callback(self, ros_data):
        self.msg.axes = ros_data.data[:ros_data.data.index(';')].split(',')
        self.msg.buttons = ros_data.data[ros_data.data.index(
            ';') + 1:].split(',')

        for i, stringy in enumerate(self.msg.buttons):
            self.msg.buttons[i] = int(stringy)
        print self.msg.buttons[7]
        for i, stringy in enumerate(self.msg.axes):
            self.msg.axes[i] = float(stringy)

        self.pub.joyPub(self.msg)


class api_publisher:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        self.joy = rospy.Publisher('joy', Joy, queue_size=1)
        print ("new pub")

    def joyPub(self, data):
        self.joy.publish(data)


def main(args):
    '''Initializes and cleanup ros node'''
    ic = api_subscriber()
    rospy.init_node('joy_publisher_unity', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS API subscriber module")


if __name__ == '__main__':
    main(sys.argv)
