#!/usr/bin/python

import roslib; roslib.load_manifest('motoman_driver')
import rospy
import math
import sensor_msgs.msg
import trajectory_msgs.msg
import copy
import rospy,sys

import moveit_commander
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor
from geometry_msgs.msg import PoseStamped, Pose 

from moveit_msgs.msg import Grasp,GripperTranslation,MoveItErrorCodes
from tf.transformations import quaternion_from_euler


class MoveItDemo:
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
	def grasp_pose(self,x,y,z,r,theta):
		g_p=PoseStamped()
		g_p.header.frame_id=self.end_effector_link
		g_p.pose.position.x=x+r*math.sin(theta)
		g_p.pose.position.y=y-r*math.cos(theta)
		g_p.pose.position.z=z+0.05
		g_p.pose.orientation.w=0.5*(math.cos(0.5*theta)-math.sin(0.5*theta))
		g_p.pose.orientation.x=-0.5*(math.cos(0.5*theta)+math.sin(0.5*theta))
		g_p.pose.orientation.y=0.5*(math.cos(0.5*theta)-math.sin(0.5*theta))
		g_p.pose.orientation.z=0.5*(math.sin(0.5*theta)+math.cos(0.5*theta))
		return g_p
		
	def cts(self,start_pose,end_pose,maxtries,exe_signal=False):
		waypoints=[]
		fraction=0.0
		attempts=0
		#maxtries_z=300
		waypoints.append(start_pose)	
		waypoints.append(end_pose)
		while fraction!=1 and attempts<maxtries:
			(plan,fraction)=self.arm.compute_cartesian_path(waypoints,0.005,0.0,True)
			attempts+=1
			if  (attempts%maxtries==0 and fraction!=1):
				rospy.loginfo("path planning failed with  " + str(fraction*100)+"% success.")
				return 0,0,0
				#signal=0
				continue
			elif  fraction==1:	
				rospy.loginfo("path compute successfully with "+str(attempts)+" attempts.")	
				if exe_signal:	self.arm.execute(plan)
				end_joint_state=plan.joint_trajectory.points[-1].positions	
				signal=1
				return plan,end_joint_state, signal
		
		
		
		
		
	def __init__(self):
		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('moveit_demo')
		scene=PlanningSceneInterface()
		self.scene_pub=rospy.Publisher('planning_scene',PlanningScene)
		self.colors=dict()
		rospy.sleep(1)
		self.arm=MoveGroupCommander('arm')
		cartesian=rospy.get_param('~cartesian',True)
		self.end_effector_link=self.arm.get_end_effector_link()
		self.arm.set_goal_position_tolerance(0.005)
		self.arm.set_goal_orientation_tolerance(0.025)
		self.arm.allow_replanning(True)
		
		self.reference_frame='base_link'
		self.arm.set_pose_reference_frame(self.reference_frame)
		self.arm.set_planning_time(5)
		
		
		#scene planning
		table_id='table'
		#cylinder_id='cylinder'
		#box1_id='box1'
		box2_id='box2'
		target_id='target_object'
		#scene.remove_world_object(box1_id)
		scene.remove_world_object(box2_id)
		scene.remove_world_object(table_id)
		scene.remove_world_object(target_id)
		
		rospy.sleep(2)

		table_ground=0.20
		table_size=[1,0.5,0.01]
		#box1_size=[0.1,0.05,0.03]
		#box2_size=[0.05,0.05,0.1]
		r_tool_size=[0.03,0.01,0.06]
		l_tool_size=[0.03,0.01,0.06]
		target_size=[0.03,0.03,0.1]
		

		table_pose=PoseStamped()
		table_pose.header.frame_id=self.reference_frame
		table_pose.pose.position.x=0
		table_pose.pose.position.y=0.75
		table_pose.pose.position.z=table_ground+table_size[2]/2.0
		table_pose.pose.orientation.w=1.0
		scene.add_box(table_id,table_pose,table_size)
		
		'''
		box1_pose=PoseStamped()
		box1_pose.header.frame_id=reference_frame
		box1_pose.pose.position.x=0.7
		box1_pose.pose.position.y=-0.2
		box1_pose.pose.position.z=table_ground+table_size[2]+box1_size[2]/2.0
		box1_pose.pose.orientation.w=1.0
		scene.add_box(box1_id,box1_pose,box1_size)

		box2_pose=PoseStamped()
		box2_pose.header.frame_id=self.reference_frame
		box2_pose.pose.position.x=0.6
		box2_pose.pose.position.y=-0.05
		box2_pose.pose.position.z=table_ground+table_size[2]+box2_size[2]/2.0
		box2_pose.pose.orientation.w=1.0
		scene.add_box(box2_id,box2_pose,box2_size)	
		'''	
		#left gripper
		l_p=PoseStamped()
		l_p.header.frame_id=self.end_effector_link
		l_p.pose.position.x=0.00
		l_p.pose.position.y=0.04
		l_p.pose.position.z=0.04
		l_p.pose.orientation.w=1
		scene.attach_box(self.end_effector_link,'l_tool',l_p,l_tool_size)	
		
		#right gripper
		r_p=PoseStamped()
		r_p.header.frame_id=self.end_effector_link
		r_p.pose.position.x=0.00
		r_p.pose.position.y=-0.04
		r_p.pose.position.z=0.04
		r_p.pose.orientation.w=1
		scene.attach_box(self.end_effector_link,'r_tool',r_p,r_tool_size)	
		
		#target
		target_pose=PoseStamped()
		target_pose.header.frame_id=self.reference_frame
		target_pose.pose.position.x=-0.4
		target_pose.pose.position.y=0.65
		target_pose.pose.position.z=table_ground+table_size[2]+target_size[2]/2.0
		target_pose.pose.orientation.x=0
		target_pose.pose.orientation.y=0
		target_pose.pose.orientation.z=0
		target_pose.pose.orientation.w=1
		scene.add_box(target_id,target_pose,target_size)	
		
		#grasp
		#grasp_pose(self,x,y,z,r,theta)
		
		#set color
		self.setColor(table_id,0.8,0,0,1.0)
		#self.setColor(box1_id,0.8,0.4,0,1.0)
		self.setColor(box2_id,0.8,0.4,0,1.0)
		self.setColor('r_tool',0.8,0,0)
		self.setColor('l_tool',0.8,0,0)
		self.setColor('target_object',0,1,0)
		self.sendColors()
		
		#motion planning
		j_ori_state=[-1.899937629699707, -0.5684762597084045, 0.46537330746650696, 2.3229329586029053, -0.057941947132349014, -1.2867668867111206, 0.2628822326660156]
		
		self.arm.set_joint_value_target(j_ori_state)
		self.arm.go()
		rospy.sleep(3)
		r=0.06
		maxtries=300
		for theta in range(0,157,16):
			theta/=100.0
			start_pose=self.arm.get_current_pose(self.end_effector_link)
			rospy.sleep(2)
			grasp_pose=self.grasp_pose(target_pose.pose.position.x,target_pose.pose.position.y,target_pose.pose.position.z,r,theta)
			if cartesian:
				(plan,e_j_state,s)=self.cts(start_pose.pose,grasp_pose.pose,maxtries)
				if s==0: continue
				#print(plan.joint_trajectory.points)
				#self.arm.set_joint_value_target(e_j_state)
				#self.arm.go()
				self.arm.execute(plan)
				rospy.sleep(5)
				#self.arm.set_named_target("initial_arm")
				#self.arm.go()
				#rospy.sleep(2)
				
				

		#remove and shut down
		scene.remove_attached_object(self.end_effector_link,'l_tool')
		scene.remove_attached_object(self.end_effector_link,'r_tool')
		moveit_commander.roscpp_shutdown()
		moveit_commander.os._exit(0)
		
		
		
		
		
		
if __name__=="__main__":
	try:
		MoveItDemo()
	except KeyboardInterrupt:
			raise
		

