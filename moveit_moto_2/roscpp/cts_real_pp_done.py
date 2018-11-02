#!/usr/bin/python

import roslib; roslib.load_manifest('motoman_driver')
import rospy
import sensor_msgs.msg
import trajectory_msgs.msg
import copy
import rospy,sys
import moveit_commander

from std_msgs.msg import Duration

from motoman_control.msg import JointAnglesDuration,JointAngles
from trajectory_msgs.msg import JointTrajectoryPoint

from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor
from geometry_msgs.msg import PoseStamped, Pose 
from copy import deepcopy
from moveit_msgs.msg import Grasp,GripperTranslation,MoveItErrorCodes
from tf.transformations import quaternion_from_euler
from robotiq_gripper_node.srv import *


class PickAndPlace:
	def setColor(self,name,r,g,b,a=0.9):
			color=ObjectColor()
			color.id=name
			color.color.r=r
			color.color.g=g
			color.color.b=b
			color.color.a=a
			self.colors[name]=color
		    
	def sendColors(self):
			p=PlanningScene()
			p.is_diff=True
			for color in self.colors.values():
				p.object_colors.append(color)
			self.scene_pub.publish(p)
			
	def setupSence(self):
		

		
		r_tool_size=[0.03,0.02,0.18]
		l_tool_size=[0.03,0.02,0.18]
		'''
		#sim table
		table_pose=PoseStamped()
		table_pose.header.frame_id=self.reference_frame
		table_pose.pose.position.x=0.75
		table_pose.pose.position.y=0.0
		table_pose.pose.position.z=self.table_ground+self.table_size[2]/2.0
		table_pose.pose.orientation.w=1.0
		self.scene.add_box(self.table_id,table_pose,self.table_size)
		'''
		#real scene table
		table_pose=PoseStamped()
		table_pose.header.frame_id=self.reference_frame
		table_pose.pose.position.x=-0.184
		table_pose.pose.position.y=0.62
		table_pose.pose.position.z=self.table_ground+self.table_size[2]/2.0
		table_pose.pose.orientation.w=1.0
		self.scene.add_box(self.table_id,table_pose,self.table_size)
		
		#left gripper
		l_p=PoseStamped()
		l_p.header.frame_id=self.arm_end_effector_link
		l_p.pose.position.x=0.00
		l_p.pose.position.y=0.057
		l_p.pose.position.z=0.09
		l_p.pose.orientation.w=1
		self.scene.attach_box(self.arm_end_effector_link,self.l_id,l_p,l_tool_size)	
		
		#right gripper
		r_p=PoseStamped()
		r_p.header.frame_id=self.arm_end_effector_link
		r_p.pose.position.x=0.00
		r_p.pose.position.y=-0.057
		r_p.pose.position.z=0.09
		r_p.pose.orientation.w=1
		self.scene.attach_box(self.arm_end_effector_link,self.r_id,r_p,r_tool_size)	
		
		
	def __init__(self):
		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('moveit_demo')
		self.scene=PlanningSceneInterface()
		self.scene_pub=rospy.Publisher('planning_scene',PlanningScene)
		self.gripperCtrl=rospy.ServiceProxy("/two_finger/gripper/gotoPositionUntilTouch",SetPosition)
		self.m2j=rospy.Publisher("/two_finger/motoman_control/move_to_joint",JointAnglesDuration,queue_size=1,latch=True)
		self.colors=dict()
		rospy.sleep(1)
		arm=MoveGroupCommander('arm')
		cartesian=rospy.get_param('~cartesian',True)
		self.arm_end_effector_link=arm.get_end_effector_link()
		arm.set_goal_position_tolerance(0.005)
		arm.set_goal_orientation_tolerance(0.025)
		arm.allow_replanning(True)
		self.reference_frame='base_link'
		arm.set_pose_reference_frame(self.reference_frame)
		arm.set_planning_time(5)	
		
		#scene planning	
		self.l_id='l_tool'
		self.r_id='r_tool'
		self.table_id='table'
		self.target_id='target_object'
		self.f_target_id='receive_container'
		self.scene.remove_world_object(self.l_id)
		self.scene.remove_world_object(self.r_id)
		self.scene.remove_world_object(self.table_id)
		self.scene.remove_world_object(self.target_id)
		#self.scene.remove_attached_object(self.arm_end_effector_link,self.target_id)
		self.scene.remove_world_object(self.f_target_id)

		self.table_ground=0.13
		self.table_size=[0.9,0.6,0.018]
		self.setupSence()
		
		target_size=[0.058,0.058,0.19]
		'''
		#target sim
		target_pose=PoseStamped()
		target_pose.header.frame_id=self.reference_frame
		target_pose.pose.position.x=0.6
		target_pose.pose.position.y=0.05
		target_pose.pose.position.z=self.table_ground+self.table_size[2]+target_size[2]/2.0
		target_pose.pose.orientation.x=0
		target_pose.pose.orientation.y=0
		target_pose.pose.orientation.z=0
		target_pose.pose.orientation.w=1
		self.scene.add_box(self.target_id,target_pose,target_size)	
		'''



		f_target_size=[0.2,0.2,0.04]
		#final target
		f_target_pose=PoseStamped()
		f_target_pose.header.frame_id=self.reference_frame
		f_target_pose.pose.position.x=-0.184+0.27
		f_target_pose.pose.position.y=0.62+0.1
		f_target_pose.pose.position.z=self.table_ground+self.table_size[2]+f_target_size[2]/2.0
		f_target_pose.pose.orientation.x=0
		f_target_pose.pose.orientation.y=0
		f_target_pose.pose.orientation.z=0
		f_target_pose.pose.orientation.w=1
		self.scene.add_box(self.f_target_id,f_target_pose,f_target_size)
		#pouring pose
		pour_pose=f_target_pose
		pour_pose.pose.position.x-=0.06
		pour_pose.pose.position.y-=0.12
		pour_pose.pose.position.z+=0.15
		#pour_pose.pose.position.y+=0.17
		pour_pose.pose.orientation.x=-0.5
		pour_pose.pose.orientation.y=-0.5
		pour_pose.pose.orientation.z=-0.5
		pour_pose.pose.orientation.w=0.5
		
		#set color
		self.setColor(self.table_id,0.8,0,0,1.0)
		self.setColor(self.f_target_id,0.8,0.4,0,1.0)
		self.setColor('r_tool',0.8,0,0)
		self.setColor('l_tool',0.8,0,0)
		self.setColor(self.target_id,0,1,0)
		self.sendColors()
		self.gripperCtrl(255)
		rospy.sleep(3)
		arm.set_named_target("initial_arm")
		arm.go()
		rospy.sleep(5)
		
		
		#j_ori_state=[0.72316974401474, 0.4691084027290344, 0.41043856739997864, -2.3381359577178955, 0.5580500364303589, 1.1085972785949707, 3.14]
		j_ori_state=[-1.899937629699707, -0.5684762597084045, 0.46537330746650696, 2.3229329586029053, -0.057941947132349014, -1.2867668867111206, 0.2628822326660156]

		#j_trans_state=[1.708616018295288, 0.9996442198753357, -0.4782222807407379, -1.7541601657867432, 0.13480804860591888, 1.1835496425628662, 2.689549684524536]
		signal= True
		arm.set_joint_value_target(j_ori_state)
		arm.go()
		rospy.sleep(3)
		#arm.set_joint_value_target(j_trans_state)
		#arm.go()
		#rospy.sleep(5)
		waypoints_1=[]
		waypoints_2=[]
		start_pick_pose=arm.get_current_pose(self.arm_end_effector_link).pose	
		if cartesian:
			msg = rospy.wait_for_message('/aruco_single/pose',PoseStamped)	
			target_pose=PoseStamped()
			target_pose.header.frame_id=self.reference_frame
			target_pose.pose.position.x=-(msg.pose.position.y)-0.28
			target_pose.pose.position.y=-msg.pose.position.x+0.81-0.072
			target_pose.pose.position.z=1.36-msg.pose.position.z+0.1
			target_pose.pose.orientation.x=0
			target_pose.pose.orientation.y=0
			target_pose.pose.orientation.z=-0
			target_pose.pose.orientation.w=1
			self.scene.add_box(self.target_id,target_pose,target_size)
			self.setColor(self.target_id,0,0.8,0)
			self.sendColors()
			#pre_g_pose
			pre_grasp_pose=PoseStamped()
			pre_grasp_pose.header.frame_id=self.reference_frame
			pre_grasp_pose.pose.position=target_pose.pose.position
			pre_grasp_pose.pose.position.y-=0.2
			pre_grasp_pose.pose.position.z+=0.01
			pre_grasp_pose.pose.orientation.x=-0.5
			pre_grasp_pose.pose.orientation.y=-0.5
			pre_grasp_pose.pose.orientation.z=-0.5
			pre_grasp_pose.pose.orientation.w=0.5
			#grasp_pose
			grasp_pose=PoseStamped()
			grasp_pose.header.frame_id=self.reference_frame
			grasp_pose.pose.position=target_pose.pose.position
			grasp_pose.pose.position.y+=0.025

			grasp_pose.pose.orientation.x=-0.5
			grasp_pose.pose.orientation.y=-0.5
			grasp_pose.pose.orientation.z=-0.5
			grasp_pose.pose.orientation.w=0.5

			g_p=PoseStamped()
			g_p.header.frame_id=self.arm_end_effector_link
			g_p.pose.position.x=0.00
			g_p.pose.position.y= -0.00
			g_p.pose.position.z=0.025
			g_p.pose.orientation.w=0.707
			g_p.pose.orientation.x=0
			g_p.pose.orientation.y=-0.707
			g_p.pose.orientation.z=0
			waypoints_1.append(start_pick_pose)	
			#waypoints_1.append(deepcopy(pre_grasp_pose.pose))
			waypoints_1.append(deepcopy(grasp_pose.pose))
			fraction=0.0
			attempts=0
		
			maxtries=300
			while fraction!=1 and attempts<maxtries:
				(plan_1,fraction)=arm.compute_cartesian_path(waypoints_1,0.01,0.0,True)
				attempts+=1
				if  (attempts %300==0 and fraction!=1):
					rospy.loginfo("path planning failed with  " + str(fraction*100)+"% success.")
					signal_pick=False
				if  fraction==1:	
					rospy.loginfo("path compute successfully with "+str(attempts)+" sttempts.")	
					rospy.loginfo("Arm moving.")
					rospy.sleep(3)
					#arm.execute(plan_1)
					p_pick_1=plan_1.joint_trajectory.points[-1]
					#print p	
					arm.set_joint_value_target(p_pick_1.positions)
					arm.go()							
					rospy.sleep(7)	
					self.gripperCtrl(0)
					rospy.sleep(3)
					self.scene.remove_world_object(self.target_id)
					#p_pick_2=plan_1.joint_trajectory.points[1]
					#print p	
					#arm.set_joint_value_target(p_pick_2.positions)
					#arm.go()							
					#rospy.sleep(4)	
					signal_pick=True
					#p=plan_1.joint_trajectory.points[-1]
					#print p	
					#arm.set_joint_value_target(p.positions)
					#arm.go()
			if signal_pick:
				start_trans_pose=arm.get_current_pose(self.arm_end_effector_link).pose
				waypoints_2.append(start_trans_pose)	
				#waypoints_1.append(deepcopy(pre_grasp_pose.pose))
				waypoints_2.append(deepcopy(pour_pose.pose))
				fraction_2=0.0
				attempts_2=0
				maxtries_2=300
				while fraction_2!=1 and attempts_2<maxtries_2:
					(plan_2,fraction_2)=arm.compute_cartesian_path(waypoints_2,0.01,0.0,True)
					attempts_2+=1
					if  (attempts_2 %300==0 and fraction_2!=1):
						rospy.loginfo("path planning failed with  " + str(fraction_2*100)+"% success.")
						
					if  fraction_2==1:
						#signal=False
						rospy.loginfo("path compute successfully with "+str(attempts_2)+" sttempts.")
						rospy.loginfo("Arm moving.")
						p_trans=plan_2.joint_trajectory.points[-1]
					#print p	
						arm.set_joint_value_target(p_trans.positions)
						arm.go()
						rospy.sleep(3)
						j_pre_pour_state=arm.get_current_joint_values()
						
						j_pour_state=j_pre_pour_state
						j_pour_state[6]-=(1.57+0.26)
						arm.set_joint_value_target(j_pour_state)
						arm.go()
		print "shaking balabala"
		rospy.sleep(1)
		print "shaking balabala"
		rospy.sleep(1)
		print "shaking balabala"
		rospy.sleep(1)
		'''
		arm.set_joint_value_target(j_pre_pour_state)
		arm.go()
		rospy.sleep(6)
		arm.set_joint_value_target(p_pick_1.positions)
		arm.go()							
		rospy.sleep(7)	
		self.gripperCtrl(255)
		arm.set_joint_value_target(j_ori_state)
		arm.go()
		rospy.sleep(5)
		'''
		
		
				
					
						
		
		'''
		
			
			#rospy.sleep(2)
			#arm.set_pose_target(pre_grasp_pose,self.arm_end_effector_link)
			#arm.go()
			#rospy.sleep(15)

			start_pick_pose=arm.get_current_pose(self.arm_end_effector_link).pose
			waypoints_1=[]
			waypoints_2=[]
			if cartesian:
				waypoints_1.append(start_pick_pose)	
				#waypoints_1.append(deepcopy(pre_grasp_pose.pose))
				waypoints_1.append(deepcopy(grasp_pose.pose))
				fraction=0.0
				attempts=0
				maxtries=300
				while fraction!=1 and attempts<maxtries:
					(plan_1,fraction)=arm.compute_cartesian_path(waypoints_1,0.01,0.0,True)
					attempts+=1
					if  (attempts %300==0 and fraction!=1):
						rospy.loginfo("path planning failed with  " + str(fraction*100)+"% success.")
						signal_1=False
					if  fraction==1:
						#signal=False
						rospy.loginfo("path compute successfully with "+str(attempts)+" sttempts.")
						rospy.loginfo("Arm moving.")
						#print plan_1.joint_trajectory.points	
						arm.execute(plan_1)	
						
						#self.gripperCtrl(0)
						rospy.sleep(7)
						self.gripperCtrl(0)
						self.scene.attach_box(self.arm_end_effector_link,self.target_id,g_p,target_size)	
				#signal=False
				start_trans_pose=arm.get_current_pose(self.arm_end_effector_link).pose
				waypoints_2.append(start_trans_pose)	
				#waypoints_1.append(deepcopy(pre_grasp_pose.pose))
				waypoints_2.append(deepcopy(pour_pose.pose))
				fraction_2=0.0
				attempts_2=0
				maxtries_2=300
				while fraction_2!=1 and attempts_2<maxtries_2:
					(plan_2,fraction_2)=arm.compute_cartesian_path(waypoints_2,0.01,0.0,True)
					attempts_2+=1
					if  (attempts_2 %300==0 and fraction_2!=1):
						rospy.loginfo("path planning failed with  " + str(fraction_2*100)+"% success.")
						#signal_1=False
					if  fraction_2==1:
						#signal=False
						rospy.loginfo("path compute successfully with "+str(attempts_2)+" sttempts.")
						rospy.loginfo("Arm moving.")
						#print plan_1.joint_trajectory.points	
						arm.execute(plan_2)	
						#self.gripperCtrl(0)
						rospy.sleep(7)
						j_state=arm.get_current_joint_values()
						j_state[6]+=(1.57+0.26)
						arm.set_joint_value_target(j_state)

		
				#if fraction==1.0:
					#rospy.loginfo("path compute successfully. Move the arm.")

					#self.m_pub_m2j.publish(place)
					#arm.execute(plan_1)
					#for i in range(len(plan_1.joint_trajectory.points)):
					#p=plan_1.joint_trajectory.points[-1]
					#place=JointAnglesDuration(JointAngles(p.positions[0],p.positions[1],p.positions[2],p.positions[3],p.positions[4],p.positions[5],p.positions[6]),rospy.Duration(5))
					#self.m2j.publish(place)
					#self.gripperCtrl(0)
					#rospy.sleep(2)
					#self.gripperCtrl(255)
					#rospy.sleep(2)
					
				'''	
					
			
		#remove and shut down
		#self.scene.remove_attached_object(self.arm_end_effector_link,'l_tool')
		#self.scene.remove_attached_object(self.arm_end_effector_link,'r_tool')
		moveit_commander.roscpp_shutdown()
		moveit_commander.os._exit(0)
		
		
		
		
		
		
if __name__=="__main__":
	try:
			PickAndPlace()
	except KeyboardInterrupt:
			raise
		
