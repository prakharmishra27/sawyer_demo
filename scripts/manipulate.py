#!/usr/bin/env python3

from geometry_msgs.msg import Pose, PoseStamped
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
import rospy
from intera_interface import CHECK_VERSION, RobotEnable, Gripper
from moveit_commander.conversions import pose_to_list
from datetime import datetime

class GripperControl:
	def __init__(self):
		self.right_gripper = Gripper('right_gripper')
		self.rs = RobotEnable(CHECK_VERSION)
	
	def init(self):
		self.right_gripper.calibrate()

	def gripper_open(self):
		self.right_gripper.open()
		rospy.sleep(2)

	def gripper_close(self):
		self.right_gripper.close()
		rospy.sleep(2)

class Manipulate:
	def __init__(self):
		joint_state_topic = ['joint_states:=/robot/joint_states']
		moveit_commander.roscpp_initialize(joint_state_topic)
	
		self.robot = moveit_commander.RobotCommander()
		self.scene = moveit_commander.PlanningSceneInterface()

		group_name = "right_arm"
		self.group = moveit_commander.MoveGroupCommander(group_name)

		self.planning_frame = self.group.get_planning_frame()
		print("============ Reference frame: %s" % self.planning_frame)

		self.eef_link = self.group.get_end_effector_link()
		print("============ End effector: %s" % self.eef_link)

		self.group_names = self.robot.get_group_names()
		print("============ Robot Groups:", self.robot.get_group_names())

		self.gripper = GripperControl()

	def all_close(self, goal, actual, tolerance):
		all_equal = True
		if type(goal) is list:
			for index in range(len(goal)):
				if abs(actual[index] - goal[index]) > tolerance:
					return False

		elif type(goal) is PoseStamped:
			return self.all_close(goal.pose, actual.pose, tolerance)

		elif type(goal) is Pose:
			return self.all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

		return True

	def get_current_state(self):
		self.group.set_pose_reference_frame("base")
		curr_pose = self.group.get_current_pose()
		print("Current Pose:")
		print("\tTranslation:")
		print(f"\t\t{curr_pose.pose.position.x}")
		print(f"\t\t{curr_pose.pose.position.y}")
		print(f"\t\t{curr_pose.pose.position.z}")
		print("\tRotation:")
		print(f"\t\t{curr_pose.pose.orientation.w}")
		print(f"\t\t{curr_pose.pose.orientation.x}")
		print(f"\t\t{curr_pose.pose.orientation.y}")
		print(f"\t\t{curr_pose.pose.orientation.z}")
		print()
		return curr_pose
	
	def save_pose(self,name,poses_to_save, suffix=None):
		recording_time = datetime.now()
		dt_string = recording_time.strftime("%d%m%Y_%H%M%S")
		save_file = ""
		if suffix is None:
			save_file = f"pose_{dt_string}.txt"
		else:
			save_file = f"pose_{suffix}.txt"
		with open(save_file,'w') as f:
			for n, p in zip(name, poses_to_save):
				f.write(f"{n}\t{p.pose.position.x}\t{p.pose.position.y}\t{p.pose.position.z}\t{p.pose.orientation.w}\t{p.pose.orientation.x}\t{p.pose.orientation.y}\t{p.pose.orientation.z}\n")

	def read_pose(self, data_file):
		name_list = []
		pose_list = []
		with open(data_file, 'r') as f:
			data = f.readlines()
			for item in data:
				each_pose = item.split('\t')
				name_list.append(each_pose[0])
				p = Pose()
				p.position.x = float(each_pose[1].strip())
				p.position.y = float(each_pose[2].strip())
				p.position.z = float(each_pose[3].strip())
				p.orientation.w = float(each_pose[4].strip())
				p.orientation.x = float(each_pose[5].strip())
				p.orientation.y = float(each_pose[6].strip())
				p.orientation.z = float(each_pose[7].strip())
				pose_list.append(p)
		assert len(name_list) == len(pose_list), "Please check data, size mismatch!"
		return name_list, pose_list


	def go_to_pose(self, pose):
		self.group.set_pose_reference_frame("base")
		self.group.set_max_velocity_scaling_factor(value= 0.25)
		self.group.set_max_acceleration_scaling_factor(value= 0.25)	
		self.group.allow_replanning(value=True)
		self.group.set_planning_time(5)	
		self.group.set_num_planning_attempts(4)				
		# self.group.set_goal_position_tolerance(value=0.5)
		curr_pose = self.group.get_current_pose()
		
		waypoints = [curr_pose.pose, pose]

		plan = RobotTrajectory()
		(plan, fraction) = self.group.compute_cartesian_path(waypoints, 0.008, 6.0, avoid_collisions = True)
		print(fraction)

		# self.group.set_pose_target(pose)
		# try:
		# 	ret = self.group.plan()
		# 	plan = ret[1]
		# except Exception as e:
		# 	print("Planning error")
		try:
			if fraction > 0.9:
				ret = self.group.execute(plan, wait=True)
			else:				
				pass
		except Exception as f:
			print("Execution error")

		self.group.stop()
		self.group.clear_pose_targets()
		current_pose = self.group.get_current_pose().pose
		
		return self.all_close(pose, current_pose, 0.01)

	def pick(self,pose_pre,pose_grasp):

		self.gripper.gripper_open()
		self.go_to_pose(pose_pre)
		rospy.sleep(3)

		self.go_to_pose(pose_grasp)	
		rospy.sleep(3)
		self.gripper.gripper_close()
		self.go_to_pose(pose_pre)

	def place(self,pose_pre,pose_release):
		
		self.go_to_pose(pose_pre)
		rospy.sleep(3)
		
		self.go_to_pose(pose_release)	
		rospy.sleep(3)
		self.gripper.gripper_open()
		self.go_to_pose(pose_pre)

	def ketchupickup(self,pose_pre, pose_grasp,pose_pick):

		self.gripper.gripper_open()
		self.go_to_pose(pose_pre)
		rospy.sleep(3)
		
		self.go_to_pose(pose_grasp)
		rospy.sleep(3)
		self.gripper.gripper_close()

		self.go_to_pose(pose_pick)

		
	def turn(self,pose_plate_pre,pose_turn):
		
		self.go_to_pose(pose_plate_pre)
		rospy.sleep(3)
		
		self.go_to_pose(pose_turn)	
		rospy.sleep(3)
		

if __name__ == "__main__":
	man = Manipulate()
	man.get_current_state()
	man.robot_zero_pose()








