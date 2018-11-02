#!/usr/bin/python

import roslib; roslib.load_manifest('motoman_driver')
import rospy
import sensor_msgs.msg
import trajectory_msgs.msg
import copy
import rospy,sys
import time

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
			
	def pose_cb(self,msg):
			rospy.loginfo("Marker at"+str(msg.pose.position))
			x0=msg.pose.position.x
			y0=msg.pose.position.y
			z0=msg.pose.position.z
			print x0,y0,z0
		    
	def sendColors(self):
			p=PlanningScene()
			p.is_diff=True
			for color in self.colors.values():
				p.object_colors.append(color)
			self.scene_pub.publish(p)	
	def __init__(self):
		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('moveit_demo')
		rospy.Subscriber('/aruco_single/pose', PoseStamped, self.pose_cb,queue_size=1)
		scene=PlanningSceneInterface()
		self.scene_pub=rospy.Publisher('planning_scene',PlanningScene)
		self.colors=dict()
		rospy.sleep(1)
		arm=MoveGroupCommander('arm')
		#gripper=MoveGroupCommander('gripper')
		end_effector_link=arm.get_end_effector_link()
		arm.set_goal_position_tolerance(0.005)
		arm.set_goal_orientation_tolerance(0.025)
		arm.allow_replanning(True)
		#gripper.set_goal_position_tolerance(0.005)
		#gripper.set_goal_orientation_tolerance(0.025)
		#gripper.allow_replanning(True)
		
		reference_frame='base_link'
		arm.set_pose_reference_frame(reference_frame)
		arm.set_planning_time(5)
		
		
		#scene planning
		table_id='table'
		#cylinder_id='cylinder'
		
		box2_id='box2'
		target_id='target_object'
		#scene.remove_world_object(box1_id)
		scene.remove_world_object(box2_id)
		scene.remove_world_object(table_id)
		scene.remove_world_object(target_id)
		
		rospy.sleep(2)

		table_ground=0.59
		table_size=[0.5,1,0.01]
		#box1_size=[0.1,0.05,0.03]
		box2_size=[0.15,0.15,0.02]
		r_tool_size=[0.05,0.04,0.22]
		l_tool_size=[0.05,0.04,0.22]
		target_size=[0.05,0.05,0.1]
		
		
		
		

		table_pose=PoseStamped()
		table_pose.header.frame_id=reference_frame
		table_pose.pose.position.x=0.7
		table_pose.pose.position.y=0.0
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
		'''
		
		box2_pose=PoseStamped()
		box2_pose.header.frame_id=reference_frame
		box2_pose.pose.position.x=0.55
		box2_pose.pose.position.y=-0.12
		box2_pose.pose.position.z=table_ground+table_size[2]+box2_size[2]/2.0
		box2_pose.pose.orientation.w=1.0
		scene.add_box(box2_id,box2_pose,box2_size)	
		
		
		target_pose=PoseStamped()
		target_pose.header.frame_id=reference_frame
		target_pose.pose.position.x=0.58
		target_pose.pose.position.y=0.05
		target_pose.pose.position.z=table_ground+table_size[2]+target_size[2]/2.0
		target_pose.pose.orientation.x=0
		target_pose.pose.orientation.y=0
		target_pose.pose.orientation.z=0
		target_pose.pose.orientation.w=1
		scene.add_box(target_id,target_pose,target_size)	
		
		#left gripper
		l_p=PoseStamped()
		l_p.header.frame_id=end_effector_link
		l_p.pose.position.x=0.00
		l_p.pose.position.y=0.06
		l_p.pose.position.z=0.11
		l_p.pose.orientation.w=1
		scene.attach_box(end_effector_link,'l_tool',l_p,l_tool_size)	
		#right gripper
		r_p=PoseStamped()
		r_p.header.frame_id=end_effector_link
		r_p.pose.position.x=0.00
		r_p.pose.position.y= -0.06
		r_p.pose.position.z=0.11
		r_p.pose.orientation.w=1
		scene.attach_box(end_effector_link,'r_tool',r_p,r_tool_size)	
		
		#grasp
		g_p=PoseStamped()
		g_p.header.frame_id=end_effector_link
		g_p.pose.position.x=0.00
		g_p.pose.position.y= -0.00
		g_p.pose.position.z=0.025
		g_p.pose.orientation.w=0.707
		g_p.pose.orientation.x=0
		g_p.pose.orientation.y=-0.707
		g_p.pose.orientation.z=0
		
		
		
		

		self.setColor(table_id,0.8,0,0,1.0)
		#self.setColor(box1_id,0.8,0.4,0,1.0)
		self.setColor(box2_id,0.8,0.4,0,1.0)
		self.setColor('r_tool',0.8,0,0)
		self.setColor('l_tool',0.8,0,0)
		self.setColor('target_object',0,1,0)
		self.sendColors()
		
		#motion planning
		arm.set_named_target("initial_arm")
		arm.go()
		rospy.sleep(2)
		
		grasp_pose=target_pose
		grasp_pose.pose.position.x-=0.15
		#grasp_pose.pose.position.z=
		grasp_pose.pose.orientation.x=0
		grasp_pose.pose.orientation.y=0.707
		grasp_pose.pose.orientation.z=0
		grasp_pose.pose.orientation.w=0.707
		

       
		#arm.set_start_state_to_current_state()
		'''
		arm.set_pose_target(grasp_pose,end_effector_link)
		traj=arm.plan()
		arm.execute(traj)
		print arm.get_current_joint_values()
		
		'''
		pre_joint_state=[0.16588150906995922, 1.7060146047438647, -0.00961761728757362, 1.8614674591892713, -2.9556667436476847, 1.7432451233907822, 3.1415]
		arm.set_joint_value_target(pre_joint_state)
		traj=arm.plan()
		arm.execute(traj)
		rospy.sleep(2)
		arm.shift_pose_target(0,0.09,end_effector_link)
		arm.go()
		rospy.sleep(2)
		
		scene.attach_box(end_effector_link,target_id,g_p,target_size)	
		rospy.sleep(2)
		
		#grasping is over , from now is pouring
		arm.shift_pose_target(2,0.15,end_effector_link)
		arm.go()
		rospy.sleep(2)
		joint_state_1=arm.get_current_joint_values()
		joint_state_1[0]-=0.17
		arm.set_joint_value_target(joint_state_1)
		arm.go()
		rospy.sleep(1)
		joint_state_2=arm.get_current_joint_values()
		joint_state_2[6]-=1.8
		arm.set_joint_value_target(joint_state_2)
		arm.go()
		rospy.sleep(1)
		
		#print arm.get_current_joint_values()
		#pouring test
		for i in range(1,5):
			joint_state_2[6]+=0.087
			arm.set_joint_value_target(joint_state_2)
			arm.go()
			time.sleep(0.05)
			
			joint_state_2[6]-=0.087
			arm.set_joint_value_target(joint_state_2)
			arm.go()
			time.sleep(0.05)
			
			print i
		
		joint_state_2[6]+=1.8
		arm.set_joint_value_target(joint_state_2)
		arm.go()
		rospy.sleep(2)
		
		joint_state_1[0]+=0.17
		arm.set_joint_value_target(joint_state_1)
		arm.go()
		rospy.sleep(1)
		arm.shift_pose_target(2,-0.15,end_effector_link)
		arm.go()
		rospy.sleep(2)
		scene.remove_attached_object(end_effector_link,target_id)
		rospy.sleep(2)

		arm.set_named_target("initial_arm")
		arm.go()
		rospy.sleep(2)

		#remove and shut down
		scene.remove_attached_object(end_effector_link,'l_tool')
		rospy.sleep(1)
		scene.remove_attached_object(end_effector_link,'r_tool')
		rospy.sleep(1)
		moveit_commander.roscpp_shutdown()
		moveit_commander.os._exit(0)
		
		
		
		
		
		
if __name__=="__main__":
	try:
		MoveItDemo()
	except KeyboardInterrupt:
			raise
		

