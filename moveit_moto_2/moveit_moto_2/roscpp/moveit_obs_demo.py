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
from geometry_msgs.msg import PoseStamped, Pose 


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
	def __init__(self):
		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('moveit_demo')
		scene=PlanningSceneInterface()
		self.scene_pub=rospy.Publisher('planning_scene',PlanningScene)
		self.colors=dict()
		rospy.sleep(1)
		arm=MoveGroupCommander('arm')
		end_effector_link=arm.get_end_effector_link()
		arm.set_goal_position_tolerance(0.01)
		arm.set_goal_orientation_tolerance(0.05)
		arm.allow_replanning(True)
		
		reference_frame='base_link'
		
		arm.set_pose_reference_frame(reference_frame)
		
		arm.set_planning_time(5)
		
		table_id='table'
		box1_id='box1'
		box2_id='box2'
		
		scene.remove_world_object(box1_id)
		scene.remove_world_object(box2_id)
		scene.remove_world_object(table_id)
		
		
		rospy.sleep(2)
		
		
		table_ground=0.35
		table_size=[0.5,0.7,0.01]
		box1_size=[0.1,0.05,0.03]
		box2_size=[0.05,0.05,0.1]
		
		
		table_pose=PoseStamped()
		table_pose.header.frame_id=reference_frame
		table_pose.pose.position.x=0.8
		table_pose.pose.position.y=0.0
		table_pose.pose.position.z=table_ground+table_size[2]/2.0
		table_pose.pose.orientation.w=1.0
		scene.add_box(table_id,table_pose,table_size)
		
		box1_pose=PoseStamped()
		box1_pose.header.frame_id=reference_frame
		box1_pose.pose.position.x=0.6
		box1_pose.pose.position.y=-0.25
		box1_pose.pose.position.z=table_ground+table_size[2]+box1_size[2]/2.0
		box1_pose.pose.orientation.w=1.0
		scene.add_box(box1_id,box1_pose,box1_size)
		
		
		box2_pose=PoseStamped()
		box2_pose.header.frame_id=reference_frame
		box2_pose.pose.position.x=0.6
		box2_pose.pose.position.y=0.25
		box2_pose.pose.position.z=table_ground+table_size[2]+box2_size[2]/2.0
		box2_pose.pose.orientation.w=1.0
		scene.add_box(box2_id,box2_pose,box2_size)		
		
		self.setColor(table_id,0.8,0,0,1.0)
		self.setColor(box1_id,0.8,0.4,0,1.0)
		self.setColor(box2_id,0.8,0.4,0,1.0)
		self.sendColors()
		
		target_pose=PoseStamped()
		target_pose.header.frame_id=reference_frame
		target_pose.pose.position.x=0.6
		target_pose.pose.position.y=-0.25
		#target_pose.pose.position.z=table_pose.pose.position.z+table_size[2]+0.15
		target_pose.pose.position.z=0.5
		target_pose.pose.orientation.y=1.57
		target_pose.pose.orientation.w=1.0
		
		arm.set_pose_target(target_pose,end_effector_link)
		arm.go()
		
		rospy.sleep(2)
		moveit_commander.roscpp_shutdown()
		moveit_commander.os._exit(0)
		
		
		
		
		
		
if __name__=="__main__":
	try:
		MoveItDemo()
	except KeyboardInterrupt:
			raise
		

