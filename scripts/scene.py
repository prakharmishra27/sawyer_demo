#!/usr/bin/env python3

import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped, Quaternion, Point

class Obstacles():
	def __init__(self):
		self.scene = moveit_commander.PlanningSceneInterface()             

	def add_box(self, model_name, position, rotation, model_size=(1,1,1)):
		box_pose = PoseStamped()
		box_pose.header.frame_id = "/base"

		box_pose.pose.position = position
		box_pose.pose.orientation = rotation  
		
		self.scene.add_box(model_name, box_pose, model_size)

if __name__ == "__main__":
	rospy.init_node('Rviz', anonymous = True)  
	rv = Obstacles()

	while not rospy.is_shutdown():
		quaternion = Quaternion()
		quaternion.w = 1.0

		pos = Point()

		pos.x = 0.7
		pos.y = 0.0
		pos.z = -0.58

		rv.add_box(model_name="table", position=pos, rotation=quaternion, model_size =(0.64,1.4,0.715))