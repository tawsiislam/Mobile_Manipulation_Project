#!/usr/bin/env python

import py_trees as pt, py_trees_ros as ptr, rospy

from behaviours_student import *
from reactive_sequence import RSequence

"""
Solution by:
Tawsiful Islam tawsiful@kth.se
Linus Lundvall llundv@kth.se
"""

class BehaviourTree(ptr.trees.BehaviourTree):

	def __init__(self):
		rospy.loginfo("Initialising behaviour tree")

		# Initiate Parameters
		self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
		self.mv_head_srv = rospy.get_param(rospy.get_name() + '/move_head_srv')
		self.pick_srv_nm = rospy.get_param(rospy.get_name() + '/pick_srv')
		self.place_srv_nm = rospy.get_param(rospy.get_name() + '/place_srv')
		self.aruco_pose_top = rospy.get_param(rospy.get_name() + '/aruco_pose_topic')
		self.pick_pose_top = rospy.get_param(rospy.get_name() + '/pick_pose_topic')
		self.place_pose_top = rospy.get_param(rospy.get_name() + '/place_pose_topic')

		#Localisation
		self.localisation_srv_nm = rospy.get_param(rospy.get_name() + '/global_loc_srv')
		self.clear_cost_srv_nm = rospy.get_param(rospy.get_name() + '/clear_costmaps_srv')
		self.amcl_estim_top = rospy.get_param(rospy.get_name() + '/amcl_estimate')

		#Navigation
		self.move_base_nm = "/move_base"
		self.clear_cost_srv = rospy.ServiceProxy(self.clear_cost_srv_nm,Empty)	# Resets where obstacles are during global planning
		
		
		b20 = movehead("up")
		b20_1 = movehead("up")
		b12 = Localisation("Localise",self.localisation_srv_nm,self.amcl_estim_top,self.cmd_vel_top,self.clear_cost_srv_nm)
		b13 = Navigation("Pick Navigation","pick",self.pick_pose_top, self.place_pose_top, self.move_base_nm,self.amcl_estim_top,self.clear_cost_srv_nm)
		b0 = tuckarm()
		b1 = movehead("down")
		b1_1 = movehead("down")
		b2 = DetectCube("Detect Cube",self.aruco_pose_top)
		b3 = PickCube("Pick Cube",self.pick_srv_nm)
		b14 = Navigation("Place Navigation","place",self.pick_pose_top,self.place_pose_top,self.move_base_nm,self.amcl_estim_top,self.clear_cost_srv_nm)
		
		b4 = pt.composites.Selector(
			name="Rotate", 
			children=[counter(29, "Facing table 2?"), go("Face table 2!", 0, -1)])
		b5 = pt.composites.Selector(
			name="Translate", 
			children=[counter(10, "At table 2?"), go("Go to table 2!", 1, 0)])
		b6 = PlaceCube("Place Cube",self.place_srv_nm)
		b7 = pt.composites.Selector(
			name="Rotate back", 
			children=[counter(29, "Facing table 1?"), go("Face table 1!", 0, -1)])
		b8 = pt.composites.Selector(
			name="Translate back", 
			children=[counter(10, "At table 1?"), go("Go to table 1!", 1, 0)])
		
		b10 = DetectCube("Detect Placing Cube",self.aruco_pose_top)
		
		b17 = RespawnCube("Respawn Cube")
		beh_list = [b20,b0,b13,b1,b2,b3,b20_1,b14,b6,b1_1,b10,b17]
		b18 = reset("Reset Cube",beh_list)
		
		b9 = RSequence(name="Going back sequence", children=[b7,b8])	# C-level fallback sequence
		b15 = RSequence(name="Reset and going back sequence",children=[b17,b18])
		
		b11 = pt.composites.Selector(name="On table fallback", children=[b10, b9])
		b16 = pt.composites.Selector(name="On table fallback and reset", children=[b10,b15])
		
		# become the tree
		tree = RSequence(name="Main sequence", children=[b20,b12,b0,b13,b1,b2,b3,b20_1,b14,b6,b1_1,b16])	# A-level
		#tree = RSequence(name="Main sequence", children=[b20,b0,b1,b2,b3,b4,b5,b6,b11])	# C-level
		super(BehaviourTree, self).__init__(tree)

		# execute the behaviour tree
		rospy.sleep(5)
		self.setup(timeout=10000)
		while not rospy.is_shutdown(): self.tick_tock(1)	

if __name__ == "__main__":


	rospy.init_node('main_state_machine')
	try:
		BehaviourTree()
	except rospy.ROSInterruptException:
		pass

	rospy.spin()
