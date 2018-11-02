#!/usr/bin/python
#coding: utf-8

import roslib; roslib.load_manifest('motoman_driver')
import sensor_msgs.msg
import trajectory_msgs.msg
import copy
import rospy,sys
import moveit_commander
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor
from geometry_msgs.msg import PoseStamped, Pose ,PoseArray,Point,Pose
from copy import deepcopy
from moveit_msgs.msg import Grasp,GripperTranslation,MoveItErrorCodes
from tf.transformations import quaternion_from_euler
from evdev import InputDevice
from select import select

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
		
		rospy.init_node('moveit_demo')
	
		#self.sub = rospy.Subscriber('/aruco_single/pose', Pose, self.update)
		#self.sub = rospy.Subscriber('/aruco_single/pose', PoseArray, self.update, queue_size=1)
		#msg = rospy.wait_for_message('/aruco_single/pose',PoseStamped)

		#grasp msg publisher
		m_pub_joint_g_1= rospy.Publisher("/ik_solution_g_1", JointTrajectoryPoint, queue_size=1, latch=True)
		m_pub_joint_g_2= rospy.Publisher("/ik_solution_g_2", JointTrajectoryPoint, queue_size=1, latch=True)
		m_pub_joint_g_3= rospy.Publisher("/ik_solution_g_3", JointTrajectoryPoint, queue_size=1, latch=True)
		m_pub_joint_g_4= rospy.Publisher("/ik_solution_g_4", JointTrajectoryPoint, queue_size=1, latch=True)
		m_pub_joint_g_5= rospy.Publisher("/ik_solution_g_5", JointTrajectoryPoint, queue_size=1, latch=True)

		#place msg publisher
		m_pub_joint_p_1= rospy.Publisher("/ik_solution_p_1", JointTrajectoryPoint, queue_size=1, latch=True)
		m_pub_joint_p_2= rospy.Publisher("/ik_solution_p_2", JointTrajectoryPoint, queue_size=1, latch=True)
		m_pub_joint_p_3= rospy.Publisher("/ik_solution_p_3", JointTrajectoryPoint, queue_size=1, latch=True)
		m_pub_joint_p_4= rospy.Publisher("/ik_solution_p_4", JointTrajectoryPoint, queue_size=1, latch=True)
		m_pub_joint_p_5= rospy.Publisher("/ik_solution_p_5", JointTrajectoryPoint, queue_size=1, latch=True)
		m_pub_joint_p_6= rospy.Publisher("/ik_solution_p_6", JointTrajectoryPoint, queue_size=1, latch=True)


		moveit_commander.roscpp_initialize(sys.argv)
		cartesian=rospy.get_param('~cartesian',True)
		scene=PlanningSceneInterface()
		self.scene_pub=rospy.Publisher('planning_scene',PlanningScene)
		self.colors=dict()
		'''
		"""set up ros publishers"""
		self.m_pub_m2j = rospy.Publisher("/two_finger/motoman_control/move_to_joint", JointAnglesDuration, queue_size=1, latch=True);
		self.m_pub_m2p = rospy.Publisher("/two_finger/motoman_control/move_to_pos", PoseDuration, queue_size=1, latch=True)
		self.m_pub_m2p_star = rospy.Publisher("/two_finger/motoman_control/move_to_pos_straight", PoseDuration, queue_size=1, latch=True)

		""" set up ros service """
		print("wait gripper service")
		rospy.wait_for_service("/two_finger/gripper/gotoPosition")	
		rospy.wait_for_service("/two_finger/gripper/gotoPositionUntilTouch")
		self.m_srv_grpCtrl = rospy.ServiceProxy("/two_finger/gripper/gotoPosition", SetPosition)
		self.m_srv_grpCtrlUntilTouch = rospy.ServiceProxy("/two_finger/gripper/gotoPositionUntilTouch", SetPosition)
		self.m_srv_grpSpeed = rospy.ServiceProxy("/two_finger/gripper/setSpeed", SetParameter)
		self.m_srv_grpTorque = rospy.ServiceProxy("/two_finger/gripper/setTorque", SetParameter)
		self.m_srv_grpTorque(20)
		self.m_srv_grpSpeed(128)
		rospy.wait_for_service("/two_finger/motoman_control/trajectory_status")
		self.m_srv_trajectoryStatus = rospy.ServiceProxy("/two_finger/motoman_control/trajectory_status", Trigger)
		'''
		
		rospy.sleep(1)
		arm=MoveGroupCommander('arm')
		arm.allow_replanning(True)
		end_effector_link=arm.get_end_effector_link()
		arm.set_goal_position_tolerance(0.03)
		arm.set_goal_orientation_tolerance(0.025)
		arm.allow_replanning(True)
		
		reference_frame='base_link'
		arm.set_pose_reference_frame(reference_frame)
		arm.set_planning_time(5)

		#scene planning
		table_id='table'
		#cylinder_id='cylinder'
		box_id='box'
		#box2_id='box2'
		target_id='target_object'
		scene.remove_world_object(box_id)
		#scene.remove_world_object(box2_id)
		scene.remove_world_object(table_id)
		#scene.remove_world_object(target_id)
		scene.remove_world_object(target_id)
		rospy.sleep(2)
		#table_ground=0.62
		table_size=[0.9,0.6,0.018]
		box_size=[0.15,0.15,0.015]
		#box2_size=[0.05,0.05,0.1]
		r_tool_size=[0.03,0.02,0.18]
		l_tool_size=[0.03,0.02,0.18]
		target_size=[0.059,0.059,0.12]
		
		table_pose=PoseStamped()
		table_pose.header.frame_id=reference_frame
		table_pose.pose.position.x=-0.05
		table_pose.pose.position.y=0.61
		table_pose.pose.position.z=0.194
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
		box2_pose.header.frame_id=reference_frame
		box2_pose.pose.position.x=0.6
		box2_pose.pose.position.y=-0.05
		box2_pose.pose.position.z=table_ground+table_size[2]+box2_size[2]/2.0
		box2_pose.pose.orientation.w=1.0
		scene.add_box(box2_id,box2_pose,box2_size)	
		'''
		#left gripper
		l_p=PoseStamped()
		l_p.header.frame_id=end_effector_link
		l_p.pose.position.x=0.00
		l_p.pose.position.y=0.057
		l_p.pose.position.z=0.09
		l_p.pose.orientation.w=1
		scene.attach_box(end_effector_link,'l_tool',l_p,l_tool_size)	
		
		#right gripper
		r_p=PoseStamped()
		r_p.header.frame_id=end_effector_link
		r_p.pose.position.x=0.00
		r_p.pose.position.y=-0.057
		r_p.pose.position.z=0.09
		r_p.pose.orientation.w=1
		scene.attach_box(end_effector_link,'r_tool',r_p,r_tool_size)	
		
		#box
		#place
		box_pose=PoseStamped()
		box_pose.header.frame_id=reference_frame
		box_pose.pose.position.x=-0.30
		box_pose.pose.position.y=0.69
		box_pose.pose.position.z=0.21
		box_pose.pose.orientation.w=1
	
		scene.add_box(box_id,box_pose,box_size)

		#place
		p_pose=box_pose
		
		p_pose.pose.position.x+=0.01
		p_pose.pose.position.y+=0.01
		p_pose.pose.position.z+=0.11
		p_pose.pose.orientation.w=-0.5
		p_pose.pose.orientation.x=-0.5
		p_pose.pose.orientation.y=-0.5
		p_pose.pose.orientation.z=0.5
		#scene.add_box(box1_id,p_pose,box2_size)
		#grasp
		g_p=PoseStamped()
		g_p.header.frame_id=end_effector_link
		g_p.pose.position.x=0
		g_p.pose.position.y=0
		g_p.pose.position.z=0.15
		g_p.pose.orientation.w=0.707
		g_p.pose.orientation.x=-0
		g_p.pose.orientation.y=-0.707
		g_p.pose.orientation.z=0.0
		
		#set color
		self.setColor(table_id,0.8,0,0,1.0)
		#self.setColor(box1_id,0.8,0.4,0,1.0)
		#self.setColor(box2_id,0.8,0.4,0,1.0)
		self.setColor('r_tool',0.8,0,0)
		self.setColor('l_tool',0.8,0,0)
		self.sendColors()
		
		#motion planning
		dev=InputDevice('/dev/input/event4')
		print "xxx"
		while not rospy.is_shutdown():
			duration=rospy.Duration(4)
			duration_s = rospy.Duration(2.5)
			select([dev],[],[])
			for event in dev.read():
				if (event.value==0) and (event.code==16):
					print "   Path planning"
					scene.remove_world_object(target_id)
					rospy.loginfo("Plz move the target")
					msg = rospy.wait_for_message('/aruco_single/pose',PoseStamped)
					target_pose=PoseStamped()
					target_pose.header.frame_id=reference_frame
					target_pose.pose.position.x=-(msg.pose.position.y)-0.28
					target_pose.pose.position.y=-msg.pose.position.x+0.81-0.068
					target_pose.pose.position.z=1.36-msg.pose.position.z+0.06
					target_pose.pose.orientation.x=0
					target_pose.pose.orientation.y=0
					target_pose.pose.orientation.z=-0
					target_pose.pose.orientation.w=1
					scene.add_box(target_id,target_pose,target_size)

					self.setColor('target_object',0,1,0)
					#pre-grasp pose
					pre_grasp_pose=PoseStamped()
					pre_grasp_pose.header.frame_id=reference_frame
					pre_grasp_pose.pose.position=target_pose.pose.position
					pre_grasp_pose.pose.position.y-=0.1
					pre_grasp_pose.pose.position.z+=0.015
					pre_grasp_pose.pose.orientation.x=-0.5
					pre_grasp_pose.pose.orientation.y=-0.5
					pre_grasp_pose.pose.orientation.z=-0.5
					pre_grasp_pose.pose.orientation.w=0.5
					#grasp pose
					grasp_pose=PoseStamped()
					grasp_pose.header.frame_id=reference_frame
					grasp_pose.pose.position=target_pose.pose.position
					grasp_pose.pose.position.y-=0.05
					grasp_pose.pose.position.z+=0.01
					
					'''
					#orientation from left
					grasp_pose.pose.orientation.x=-0.000149
					grasp_pose.pose.orientation.y=-0.707
					grasp_pose.pose.orientation.z=0
					grasp_pose.pose.orientation.w=0.707
					'''
					#orientation from forward
					grasp_pose.pose.orientation.x=-0.5
					grasp_pose.pose.orientation.y=-0.5
					grasp_pose.pose.orientation.z=-0.5
					grasp_pose.pose.orientation.w=0.5

					
					
					arm.set_named_target("initial_arm")
					arm.go()
					rospy.sleep(2)
					start_pose=arm.get_current_pose(end_effector_link).pose
					#initial waypoints list
					waypoints_1=[]
					waypoints_2=[]
					waypoints_3=[]
					waypoints_4=[]
					if cartesian:
						waypoints_1.append(start_pose)	
						waypoints_1.append(deepcopy(pre_grasp_pose.pose))
						fraction=0.0
						maxtries=300
						attempts=0
						#arm.set_start_state__1to_current_state()
						#plan the cartesian path connecting waypoints
						while fraction<1.0 and attempts<maxtries:
							(plan_1,fraction)=arm.compute_cartesian_path(waypoints_1,0.01,0.0,True)
							attempts+=1
							if  (attempts %300==0 and fraction!=1.0):
								rospy.loginfo("path planning failed with only  " + str(fraction*100)+ "% success after  "+ str(maxtries)+" attempts")
								continue
							if fraction==1.0:
								rospy.loginfo("path compute successfully. Move the arm.")
								#place = JointAnglesDuration(JointAngles(plan.joint_trajectory.points[len(plan.joint_trajectory)-1].positions), duration)
								#self.m_pub_m2j.publish(place)
								print
								arm.execute(plan_1)
								m_pub_joint_g_1.publish(plan_1.joint_trajectory.points[-1])
								m_pub_joint_g_2.publish(plan_1.joint_trajectory.points[-2])
								m_pub_joint_g_3.publish(plan_1.joint_trajectory.points[-3])
								m_pub_joint_g_4.publish(plan_1.joint_trajectory.points[-5])
								m_pub_joint_g_5.publish(plan_1.joint_trajectory.points[-8])
								rospy.loginfo("path execution complete.	")
								scene.attach_box(end_effector_link,target_id,g_p,target_size)	
								
								rospy.sleep(5)
					
					g_pose=arm.get_current_pose(end_effector_link)
					if cartesian:
						waypoints_2.append(g_pose.pose)	
						waypoints_2.append(deepcopy(p_pose.pose))
						fraction=0.0
						maxtries=300
						attempts=0
						#arm.set_start_state_to_current_state()
						#plan the cartesian path connecting waypoints
						while fraction<1.0 and attempts<maxtries:
							(plan_2,fraction)=arm.compute_cartesian_path(waypoints_2,0.01,0.0,True)
							attempts+=1
							if  (attempts %300==0 and fraction!=1.0):
								rospy.loginfo("path planning failed with only  " + str(fraction*100)+ "% success after  "+ str(maxtries)+" attempts")
								continue
							if fraction==1.0:
								rospy.loginfo("path compute successfully. Move the arm.")
								#place = JointAnglesDuration(JointAngles(plan.joint_trajectory.points[len(plan.joint_trajectory)-1].positions), duration)
								#self.m_pub_m2j.publish(place)
								arm.execute(plan_2)
								m_pub_joint_p_1.publish(plan_2.joint_trajectory.points[5])
								m_pub_joint_p_1.publish(plan_2.joint_trajectory.points[len(plan_2.joint_trajectory.points)/5])
								m_pub_joint_p_1.publish(plan_2.joint_trajectory.points[len(plan_2.joint_trajectory.points)/2])
								m_pub_joint_p_1.publish(plan_2.joint_trajectory.points[-len(plan_2.joint_trajectory.points)/5])
								m_pub_joint_p_1.publish(plan_2.joint_trajectory.points[5])
								m_pub_joint_p_1.publish(plan_2.joint_trajectory.points[1])
								rospy.loginfo("path execution complete.	")
								rospy.sleep(5)
								
								#m_pub_joint_g.publish(arm.get_current_joint_value())			
													
							
				
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
		scene.remove_world_object(target_id)
		scene.remove_world_object(table_id)
		scene.remove_attached_object(end_effector_link,'l_tool')
		scene.remove_attached_object(end_effector_link,'r_tool')
		moveit_commander.roscpp_shutdown()
		moveit_commander.os._exit(0)

if __name__=="__main__":
	try:
		MoveItDemo()
	except KeyboardInterrupt:
			raise
		

