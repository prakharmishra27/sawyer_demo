#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Pose
from manipulate import Manipulate

if __name__ == "__main__":

	rospy.init_node('sawyerdemo', anonymous=True)
	manip = Manipulate()
	rate = rospy.Rate(10)

	name_list = []
	pose_list = []
	save_key = False

	name_list = ["bun_pre", "bun_pick","patty_pre", "patty_pick", "top_bun_pre", "top_bun_pick", "plate_pre", "plate_put"]

	# name_list = ["bun_pre", "bun_pick","patty_pre", "patty_pick", "top_bun_pre", "top_bun_pick", "plate_pre", "plate_put", "ketchup_pre", "ketchup_grasp","ketchup_pick","plate_pre", "turn"]
	# name_list = ["bun_pre", "bun_pick","patty_pre", "patty_pick","lett_tom_pre", "lett_tom_pick","top_bun_pre", "top_bun_pick","ketchup_pre", "ketchup_pick","plate_pre", "plate_put","ketchup_turn"]
	pose_list = [PoseStamped() for _ in range(len(name_list))]
	curr_save = 0

	while not rospy.is_shutdown():
		if save_key is False:
			print(f"Move robot to {name_list[curr_save]}")
		else:
			print(f"Saving pose for {name_list[curr_save]}")
			pose_list[curr_save] = manip.get_current_state()
			curr_save += 1
			save_key = False
			if curr_save == len(name_list):
				break
		
		save_input = input("Save pose?")
		if 'y' in save_input.lower():
			save_key = True
		rate.sleep()
	
	# manip.save_pose(name_list, pose_list, suffix="testing5")
	# rospy.sleep(3)

	name_list2, pose_list2 = manip.read_pose("pose_testing5.txt")

	# pick and place bun_bottom	
	manip.pick(pose_list2[0],pose_list2[1])	
	manip.place(pose_list2[6],pose_list2[7])	
	

	# pick and place patty	
	manip.pick(pose_list2[2],pose_list2[3])	
	manip.place(pose_list2[6],pose_list2[7])	
	
	
	
	# pick and place top bun	
	manip.pick(pose_list2[4],pose_list2[5])	
	manip.place(pose_list2[6],pose_list2[7])	
	
	
	# # pick and pour ketchup		
	# manip.ketchupickup(pose_list2[8],pose_list2[9],pose_list2[10])	
	# manip.turn(pose_list2[11],pose_list2[12])	
	

	