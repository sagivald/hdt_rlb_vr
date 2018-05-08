#!/usr/bin/env python

import rospy
import roslib
#from ein.msg import EinState
from std_msgs.msg import String
from sensor_msgs.msg import JointState


class unityNode:
	def __init__(self):
		rospy.init_node("unityNode",anonymous=False)

		self.pub = rospy.Publisher("ros_unity",String,queue_size=10)
		self.rate = rospy.Rate(120)

		self.joint_angles_sub = rospy.Subscriber("/joint_states",JointState,self.joint_angles_callback)

		#Real Robot state	
		self.angles = [0,0,0,0,0,0,0,0,0,0]

		while not rospy.is_shutdown():
			pubString = self.messageBuilder()
			#pubString = "Cats"
			self.pub.publish(pubString)
			self.rate.sleep()

	def messageBuilder(self):
		msg = ""
		for joint in range(len(self.angles)):
			msg += str(self.angles[joint])+","
		return msg

		
	def XMLSetup(self):
		#preString = "<textarea rows=\"20\" cols=\"40\" style=\"border:none;\">"
		#xmlString = preString + "<rosUnity>"
		xmlString = "<rosUnity>"
		for joint in range(len(self.angles)):
			xmlString += "<r_"+str(joint)+">"+str(self.angles[joint])+"</r_"+str(joint)+">"
		xmlString += "</rosUnity>" #+ "</textarea>"	
		#print xmlString
		return xmlString

	def joint_angles_callback(self,message):
		self.angles = message.position
		

uN = unityNode()
