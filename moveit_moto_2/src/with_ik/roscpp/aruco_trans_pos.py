#!/usr/bin/python

import roslib; roslib.load_manifest('motoman_driver')
import rospy
import sensor_msgs.msg
import trajectory_msgs.msg
import copy
import rospy,sys
import moveit_commander
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor
from geometry_msgs.msg import PoseStamped, Pose ,PoseArray
from copy import deepcopy
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
						
	def  update(self,msg):
		#self.data=msg
		self.target_pose=PoseStamped()
		self.target_pose.header.frame_id=self.reference_frame
		self.target_pose.pose.position.x=msg.pose.position.x
		self.target_pose.pose.position.y=msg.pose.position.y
		self.target_pose.pose.position.z=msg.pose.position.z
		#target_pose.pose.position.z=table_ground+table_size[2]+target_size[2]/2.0
		self.target_pose.pose.orientation.x=0
		self.target_pose.pose.orientation.y=0
		self.target_pose.pose.orientation.z=0
		self.target_pose.pose.orientation.w=1
		#self.scene.add_box(self.target_id,self.target_pose,self.target_size)
				
	def __init__(self):
		rospy.init_node('moveit_demo')
		#self.sub = rospy.Subscriber('/aruco_single/pose', Pose, self.update)
		#self.target_pose=PoseStamped()
		self.sub = rospy.Subscriber('/aruco_single/pose', PoseStamped, self.update, queue_size=1)
		
		#msg = rospy.wait_for_message('/aruco_single/pose',PoseStamped)
		moveit_commander.roscpp_initialize(sys.argv)
		cartesian=rospy.get_param('~cartesian',True)
		self.scene=PlanningSceneInterface()
		self.scene.add_box(self.target_id,self.target_pose,self.target_size)
		self.scene_pub=rospy.Publisher('planning_scene',PlanningScene)
		self.colors=dict()
		rospy.sleep(1)
		arm=MoveGroupCommander('arm')
		arm.allow_replanning(True)
		end_effector_link=arm.get_end_effector_link()
		arm.set_goal_position_tolerance(0.03)
		arm.set_goal_orientation_tolerance(0.025)
		arm.allow_replanning(True)
		
		self.reference_frame='base_link'
		arm.set_pose_reference_frame(self.reference_frame)
		arm.set_planning_time(5)

		#scene planning
		#table_id='table'
		self.target_id='target_object'
		self.scene.remove_world_object(self.target_id)
		rospy.sleep(2)
		#table_ground=0.68
		#table_size=[0.5,1,0.01]
		#box1_size=[0.1,0.05,0.03]
		#box2_size=[0.05,0.05,0.1]
		r_tool_size=[0.03,0.01,0.06]
		l_tool_size=[0.03,0.01,0.06]
		self.target_size=[0.05,0.05,0.1]
		
		grasp_pose=self.target_pose
		grasp_pose.pose.position.x-=0.06
		grasp_pose.pose.orientation.x=0
		grasp_pose.pose.orientation.y=0.707
		grasp_pose.pose.orientation.z=0
		grasp_pose.pose.orientation.w=0.707
		
		#left gripper
		l_p=PoseStamped()
		l_p.header.frame_id=end_effector_link
		l_p.pose.position.x=0.00
		l_p.pose.position.y=0.04
		l_p.pose.position.z=0.04
		l_p.pose.orientation.w=1
		self.scene.attach_box(end_effector_link,'l_tool',l_p,l_tool_size)	
		
		#right gripper
		r_p=PoseStamped()
		r_p.header.frame_id=end_effector_link
		r_p.pose.position.x=0.00
		r_p.pose.position.y=-0.04
		r_p.pose.position.z=0.04
		r_p.pose.orientation.w=1
		self.scene.attach_box(end_effector_link,'r_tool',r_p,r_tool_size)	
		
		#grasp
		g_p=PoseStamped()
		g_p.header.frame_id=end_effector_link
		g_p.pose.position.x=0.00
		g_p.pose.position.y=-0.00
		g_p.pose.position.z=0.025
		g_p.pose.orientation.w=0.707
		g_p.pose.orientation.x=0
		g_p.pose.orientation.y=-0.707
		g_p.pose.orientation.z=0
		
		#set color
		#self.setColor(table_id,0.8,0,0,1.0)
		#self.setColor(box1_id,0.8,0.4,0,1.0)
		#self.setColor(box2_id,0.8,0.4,0,1.0)
		self.setColor('r_tool',0.8,0,0)
		self.setColor('l_tool',0.8,0,0)
		self.setColor('target_object',0,1,0)
		self.sendColors()
		
		#motion planning
		
		arm.set_named_target("initial_arm")
		arm.go()
		rospy.sleep(2)
		start_pose=arm.get_current_pose(end_effector_link).pose
		

if __name__=="__main__":
	try:
		MoveItDemo()
	except KeyboardInterrupt:
			raise
		

