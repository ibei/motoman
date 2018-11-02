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
	def __init__(self):
		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('moveit_demo')
		scene=PlanningSceneInterface()
		self.scene_pub=rospy.Publisher('planning_scene',PlanningScene)
		self.colors=dict()
		rospy.sleep(1)
		arm=MoveGroupCommander('arm')
		gripper=MoveGroupCommander('gripper')
		end_effector_link=arm.get_end_effector_link()
		arm.set_goal_position_tolerance(0.005)
		arm.set_goal_orientation_tolerance(0.025)
		arm.allow_replanning(True)
		gripper.set_goal_position_tolerance(0.005)
		gripper.set_goal_orientation_tolerance(0.025)
		gripper.allow_replanning(True)
		
		reference_frame='base_link'
		arm.set_pose_reference_frame(reference_frame)
		arm.set_planning_time(5)


		r_tool_size=[0.03,0.01,0.06]
		l_tool_size=[0.03,0.01,0.06]

		
		#left gripper
		l_p=PoseStamped()
		l_p.header.frame_id=end_effector_link
		l_p.pose.position.x=0.00
		l_p.pose.position.y=0.04
		l_p.pose.position.z=0.04
		l_p.pose.orientation.w=1
		scene.attach_box(end_effector_link,'l_tool',l_p,l_tool_size)	
		#right gripper
		r_p=PoseStamped()
		r_p.header.frame_id=end_effector_link
		r_p.pose.position.x=0.00
		r_p.pose.position.y=-0.04
		r_p.pose.position.z=0.04
		r_p.pose.orientation.w=1
		scene.attach_box(end_effector_link,'r_tool',r_p,r_tool_size)	

		grasp_pose=target_pose
		grasp_pose.pose.position.x-=0.15
		#grasp_pose.pose.position.z=
		grasp_pose.pose.orientation.x=0
		grasp_pose.pose.orientation.y=0.707
		grasp_pose.pose.orientation.z=0
		grasp_pose.pose.orientation.w=0.707

		arm.set_start_state_to_current_state()
		arm.set_pose_target(grasp_pose,end_effector_link)
		traj=arm.plan()
		arm.execute(traj)
		rospy.sleep(2)
		#arm.shift_pose_target(4,1.57,end_effector_link)
		#arm.go()
		#rospy.sleep(2)
		arm.shift_pose_target(0,0.11,end_effector_link)
		arm.go()
		rospy.sleep(2)
		saved_target_pose=arm.get_current_pose(end_effector_link)
		#arm.set_named_target("initial_arm2")
		
		#grasp
		
		scene.attach_box(end_effector_link,target_id,g_p,target_size)	
		rospy.sleep(2)
		
		#grasping is over , from now is placing 
		arm.shift_pose_target(2,0.15,end_effector_link)
		arm.go()
		rospy.sleep(2)
		arm.shift_pose_target(1,-0.2,end_effector_link)
		arm.go()
		rospy.sleep(2)
		
		#pouring test
		arm.shift_pose_target(3,1.57,end_effector_link)
		rospy.sleep(2)
		arm.shift_pose_target(3,-1.57,end_effector_link)
		rospy.sleep(2)
		#gripper.shift_pose_target(3,1.57,end_effector_link)
		
		arm.set_pose_target(saved_target_pose)
		arm.go()
		rospy.sleep(2)


		scene.remove_attached_object(end_effector_link,'l_tool')
		scene.remove_attached_object(end_effector_link,'r_tool')
		moveit_commander.roscpp_shutdown()
		moveit_commander.os._exit(0)
		
		
		
		
		
		
if __name__=="__main__":
	try:
		MoveItDemo()
	except KeyboardInterrupt:
			raise
		

