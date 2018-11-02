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

'''
class marker(object):
	def __init__(self):
		self.sub = rospy.Subscriber('/aruco_single/pose', PoseArray, self.update, queue_size=1)
		self.pose = None

	def update(self, msg):
		pose = msg.poses[0]
		self.pose = [pose.position.x,
					pose.position.y,
					pose.position.z,
		1,0,0,0]
		return self.pose
'''
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
		self.target_pose.header.frame_id=self.reference_frame
		self.target_pose.pose.position.x=msg.pose.position.x
		self.target_pose.pose.position.y=msg.pose.position.y
		self.target_pose.pose.position.z=msg.pose.position.z
		#target_pose.pose.position.z=table_ground+table_size[2]+target_size[2]/2.0
		self.target_pose.pose.orientation.x=0
		self.target_pose.pose.orientation.y=0
		self.target_pose.pose.orientation.z=0
		self.target_pose.pose.orientation.w=1
		self.scene.add_box(self.target_id,self.target_pose,self.target_size)
		
			
	def __init__(self):
		
		rospy.init_node('moveit_demo')
		#self.sub = rospy.Subscriber('/aruco_single/pose', Pose, self.update)
		self.target_pose=PoseStamped()
		self.sub = rospy.Subscriber('/aruco_single/pose', PoseStamped, self.update, queue_size=1)
		#msg = rospy.wait_for_message('/aruco_single/pose',PoseStamped)
		moveit_commander.roscpp_initialize(sys.argv)
		cartesian=rospy.get_param('~cartesian',True)
		self.scene=PlanningSceneInterface()
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
		'''
		table_pose=PoseStamped()
		table_pose.header.frame_id=reference_frame
		table_pose.pose.position.x=0.75
		table_pose.pose.position.y=0.0
		table_pose.pose.position.z=table_ground+table_size[2]/2.0
		table_pose.pose.orientation.w=1.0
		scene.add_box(table_id,table_pose,table_size)

		box1_pose=PoseStamped()
		box1_pose.header.frame_id=reference_frame
		box1_pose.pose.position.x=0.7
		box1_pose.pose.position.y=-0.2
		box1_pose.pose.position.z=table_ground+table_size[2]+box1_size[2]/2.0
		box1_pose.pose.orientation.w=1.0
		scene.add_box(box1_id,box1_pose,box1_size)

		box2_pose=PoseStamped()
		box2_pose.header.frame_id=reference_frame
		box2_pose.pose.position.x=0.6
		box2_pose.pose.position.y=-0.05
		box2_pose.pose.position.z=table_ground+table_size[2]+box2_size[2]/2.0
		box2_pose.pose.orientation.w=1.0
		scene.add_box(box2_id,box2_pose,box2_size)	
		'''
		#target pose#
		#msg = rospy.wait_for_message('/aruco_single/pose',PoseStamped)
		'''
		self.target_pose=PoseStamped()
		self.target_pose.header.frame_id=reference_frame
		self.target_pose.pose.position.x=self.data.pose.position.x
		self.target_pose.pose.position.y=self.data.pose.position.y
		self.target_pose.pose.position.z=self.data.pose.position.z
		#target_pose.pose.position.z=table_ground+table_size[2]+target_size[2]/2.0
		self.target_pose.pose.orientation.x=0
		self.target_pose.pose.orientation.y=0
		self.target_pose.pose.orientation.z=0
		self.target_pose.pose.orientation.w=1
		scene.add_box(target_id,self.target_pose,target_size)
		'''	
		
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
		
		

		#initial waypoints list
		waypoints=[]
		if cartesian:
			waypoints.append(start_pose)	
			waypoints.append(deepcopy(grasp_pose.pose))
			fraction=0.0
			maxtries=100
			attempts=0
			#arm.set_start_state_to_current_state()
			#plan the cartesian path connecting waypoints
			while fraction<1.0 and attempts<maxtries:
				(plan,fraction)=arm.compute_cartesian_path(waypoints,0.01,0.0,True)
				attempts+=1
				if attempts %20==0:
					if (attempts<100):
						rospy.loginfo("still trying after"+str(attempts)+" attempts...")
					else : 
						rospy.loginfo("Finished after  "+str(attempts)+" attempts...")

			if fraction==1.0:
				rospy.loginfo("path compute successfully. Move the arm.")
				arm.execute(plan)
				rospy.loginfo("path execution complete.	")
				rospy.sleep(2)				
			else:
				rospy.loginfo("path planning failed with only  " + str(fraction*100)+ "% success after  "+ str(maxtries)+" attempts")
		#arm.set_named_target("initial_arm")
		#arm.go()
		#rospy.sleep(1)
		'''
		arm.set_start_state_to_current_state()
		arm.set_pose_target(grasp_pose,end_effector_link)
		traj=arm.plan()
		arm.execute(traj)
		rospy.sleep(2)
		print arm.get_current_joint_values()
		#arm.shift_pose_target(4,1.57,end_effector_link)
		#arm.go()
		#rospy.sleep(2)
		arm.shift_pose_target(0,0.11,end_effector_link)
		arm.go()
		rospy.sleep(2)
		print arm.get_current_joint_values()
		saved_target_pose=arm.get_current_pose(end_effector_link)
		#arm.set_named_target("initial_arm2")
		
		#grasp
		scene.attach_box(end_effector_link,target_id,g_p,target_size)	
		rospy.sleep(2)
		
		#grasping is over , from now is placing 
		arm.shift_pose_target(2,0.15,end_effector_link)
		arm.go()
		rospy.sleep(2)
		print arm.get_current_joint_values()
		arm.shift_pose_target(1,-0.2,end_effector_link)
		arm.go()
		rospy.sleep(2)
		print arm.get_current_joint_values()
		arm.shift_pose_target(2,-0.15,end_effector_link)
		arm.go()
		rospy.sleep(2)
		print arm.get_current_joint_values()
		scene.remove_attached_object(end_effector_link,target_id)
		rospy.sleep(2)
		#arm.set_pose_target(saved_target_pose,end_effector_link)
		#arm.go()
		#rospy.sleep(2)
		
		arm.set_named_target("initial_arm1")
		arm.go()
		rospy.sleep(2)
        '''
		#remove and shut down
		self.scene.remove_attached_object(end_effector_link,'l_tool')
		self.scene.remove_attached_object(end_effector_link,'r_tool')
		moveit_commander.roscpp_shutdown()
		moveit_commander.os._exit(0)

if __name__=="__main__":
	try:
		MoveItDemo()
	except KeyboardInterrupt:
			raise
		

