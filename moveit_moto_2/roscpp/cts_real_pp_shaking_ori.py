#!/usr/bin/python

import roslib; roslib.load_manifest('motoman_driver')
import rospy
import sensor_msgs.msg
import trajectory_msgs.msg
import copy
import rospy,sys
import moveit_commander

from std_msgs.msg import Duration,Float64

from motoman_control.msg import JointAnglesDuration,JointAngles
from trajectory_msgs.msg import JointTrajectoryPoint

from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor
from geometry_msgs.msg import PoseStamped, Pose 
from copy import deepcopy
from moveit_msgs.msg import Grasp,GripperTranslation,MoveItErrorCodes
from tf.transformations import quaternion_from_euler
from robotiq_gripper_node.srv import *

from cubic_hermite_spline import TCubicHermiteSpline
#from cubic_hermite_spline import *
import geometry_msgs.msg
import numpy as np
import math
import random
import numpy.linalg as la


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

	
	def add_point(self,traj, time, positions, velocities=None):
        	point= trajectory_msgs.msg.JointTrajectoryPoint()
        	point.positions= copy.deepcopy(positions)
        	if velocities is not None:
         	   point.velocities= copy.deepcopy(velocities)
          	   point.time_from_start= rospy.Duration(time)
           	   traj.points.append(point)
	def FollowQTraj(self,q_traj, t_traj):
    		assert(len(q_traj)==len(t_traj))

    		#Insert current position to beginning.
    		if t_traj[0]>1.0e-2:
      			 t_traj.insert(0,0.0)
     			 q_traj.insert(0,self.Q(arm=arm))
    
    		self.dq_traj=self.QTrajToDQTraj(q_traj, t_traj)
    		#self.traj= self.ToROSTrajectory(self.JointNames(), q_traj, t_traj, dq_traj)  #, dq_traj

    		#print traj
    		#self.sub_jpc.publish(self.ToROSTrajectory(self.JointNames(), q_traj, t_traj, dq_traj))
    

  	def QTrajToDQTraj(self,q_traj, t_traj):
    		dof= len(q_traj[0])

  		#Modeling the trajectory with spline.
    		splines= [TCubicHermiteSpline() for d in range(dof)]
    		for d in range(len(splines)):
      			data_d= [[t,q[d]] for q,t in zip(q_traj,t_traj)]
      			splines[d].Initialize(data_d, tan_method=splines[d].CARDINAL, c=0.0, m=0.0)

  		#NOTE: We don't have to make spline models as we just want velocities at key points.
  		#  They can be obtained by computing tan_method, which will be more efficient.         with_tan=True

   		dq_traj= []
    		for t in t_traj:
      			dq= [splines[d].Evaluate(t,with_tan=True)[1] for d in range(dof)]
      			dq_traj.append(dq)
   		#print dq_traj
    		return dq_traj


  	def JointNames(self):
    		#arm= 0
    		return self.joint_names[0]      


   
  	def ROSGetJTP(self,q,t,dq=None):
    		jp= trajectory_msgs.msg.JointTrajectoryPoint()
    		jp.positions= q
    		jp.time_from_start= rospy.Duration(t)
    		if dq is not None:  jp.velocities= dq
    		return jp


 

  	def ToROSTrajectory(self,joint_names, q_traj, t_traj, dq_traj=None):
    		assert(len(q_traj)==len(t_traj))
    		if dq_traj is not None:  (len(dq_traj)==len(t_traj))
    		#traj= trajectory_msgs.msg.JointTrajectory()
	    	self.traj.joint_names= joint_names
	    	if dq_traj is not None:
	     	 self.traj.points= [self.ROSGetJTP(q,t,dq) for q,t,dq in zip(q_traj, t_traj, dq_traj)]
	    	else:
	     	 self.traj.points= [self.ROSGetJTP(q,t) for q,t in zip(q_traj, t_traj)]
	    	self.traj.header.stamp= rospy.Time.now()
	   	#print self.traj
	   	return self.traj

  	def SmoothQTraj(self,q_traj):
    		if len(q_traj)==0:  return
    		q_prev= np.array(q_traj[0])
    		q_offset= np.array([0]*len(q_prev))
    		for q in q_traj:
      			q_diff= np.array(q) - q_prev
      			for d in range(len(q_prev)):
        			if q_diff[d]<-math.pi:  q_offset[d]+=1
        			elif q_diff[d]>math.pi:  q_offset[d]-=1
      			q_prev= copy.deepcopy(q)
      			q[:]= q+q_offset*2.0*math.pi
	def cts(self,start_pose,end_pose,maxtries,exe_signal=False):
		waypoints=[]
		fraction=0.0
		attempts=0
		#maxtries_z=300
		waypoints.append(start_pose)	
		waypoints.append(end_pose)
		while fraction!=1 and attempts<maxtries:
			(plan,fraction)=self.arm.compute_cartesian_path(waypoints,0.01,0.0,True)
			attempts+=1
			if  (attempts %maxtries==0 and fraction!=1):
				rospy.loginfo("path planning failed with  " + str(fraction*100)+"% success.")
				signal=False
			elif  fraction==1:	
				rospy.loginfo("path compute successfully with "+str(attempts)+" attempts.")	
				if exe_signal:	self.arm.execute(plan)
				end_joint_state=plan.joint_trajectory.points[-1].positions	
				signal=True
				return plan,end_joint_state, signal
	# shaking function: 
	# freq : shaking freqence
	# times : shaking time per action  
	def shaking(self,start_joint_state,end_joint_state,freq,times):
		q_traj=[start_joint_state]
	    	t_traj=[0.0]
	    	for i in range(times):
			q_traj.append(end_joint_state)
			t_traj.append(t_traj[-1]+0.5/freq)
			q_traj.append(start_joint_state)
			t_traj.append(t_traj[-1]+0.5/freq)
	    	self.FollowQTraj(q_traj,t_traj)
	   	self.sub_jpc.publish(self.ToROSTrajectory(self.JointNames(), q_traj, t_traj, self.dq_traj))
		rospy.sleep(5)
		
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
		pub_traj= rospy.Publisher('/joint_path_command', trajectory_msgs.msg.JointTrajectory, queue_size=10)
		self.scene_pub=rospy.Publisher('planning_scene',PlanningScene)
		self.gripperCtrl=rospy.ServiceProxy("/two_finger/gripper/gotoPositionUntilTouch",SetPosition)
		#self.m2j=rospy.Publisher("/two_finger/motoman_control/move_to_joint",JointAnglesDuration,queue_size=1,latch=True)
		self.colors=dict()
		rospy.sleep(1)
		self.arm=MoveGroupCommander('arm')
		cartesian=rospy.get_param('~cartesian',True)
		self.arm_end_effector_link=self.arm.get_end_effector_link()
		self.arm.set_goal_position_tolerance(0.005)
		self.arm.set_goal_orientation_tolerance(0.025)
		self.arm.allow_replanning(True)
		self.reference_frame='base_link'
		self.arm.set_pose_reference_frame(self.reference_frame)
		self.arm.set_planning_time(5)
		#shaking
		self.joint_names= [[]]
    		self.joint_names[0]= rospy.get_param('controller_joint_names')
    		self.traj= trajectory_msgs.msg.JointTrajectory()
    		self.sub_jpc= rospy.Publisher('/joint_path_command', trajectory_msgs.msg.JointTrajectory,queue_size=10)	
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
		
		joint_names= ['joint_'+jkey for jkey in ('s','l','e','u','r','b','t')]
   		joint_names= rospy.get_param('controller_joint_names')
		traj= trajectory_msgs.msg.JointTrajectory()
		traj.joint_names= joint_names
		
		target_size=[0.058,0.058,0.19]
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
		pour_pose.pose.position.x-=0.04
		pour_pose.pose.position.y-=0.135
		pour_pose.pose.position.z+=0.18
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
		self.arm.set_named_target("initial_arm")
		self.arm.go()
		rospy.sleep(5)
		
		#pre pose
		j_ori_state=[-1.899937629699707, -0.5684762597084045, 0.46537330746650696, 2.3229329586029053, -0.057941947132349014, -1.2867668867111206, 0.2628822326660156]
		self.arm.set_joint_value_target(j_ori_state)
		self.arm.go()
		rospy.sleep(3)
		#get target pose
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
		
		rotate_ori=1.57+0.36
		
		start_pick_pose=self.arm.get_current_pose(self.arm_end_effector_link).pose
		start_joint_state=self.arm.get_current_joint_values()
		if cartesian:
			plan_pick,end_joint_state_pick,signal_pick=self.cts(start_pick_pose,grasp_pose.pose,300)
			if signal_pick:
				rospy.loginfo("picking  is ok!")
				self.arm.set_joint_value_target(end_joint_state_pick)
				self.arm.go()
				rospy.sleep(5)
				self.gripperCtrl(0)
				rospy.sleep(2)
				self.scene.remove_world_object(self.target_id)
				signal_trans=True
		if signal_trans:
			start_trans_pose=self.arm.get_current_pose(self.arm_end_effector_link).pose
			if cartesian:
				plan_trans,end_joint_state_trans,signal_trans=self.cts(start_trans_pose,pour_pose.pose,300)
				if signal_trans:
					rospy.loginfo("trans is ok!")
					self.arm.set_joint_value_target(end_joint_state_trans)
					self.arm.go()
					rospy.sleep(4)
					pre_pour_state=self.arm.get_current_joint_values()
					pour_state=pre_pour_state
					pour_state[6]-=rotate_ori
					self.arm.set_joint_value_target(pour_state)
					self.arm.go()
					rospy.sleep(2)
					shaking_signal=True
		
		rotate=0
		shake_times=0
		freq=2.7
		incre_x=0.1
		incre_z=0.05
		while shaking_signal:
			#shaking process
			start_shaking_pose=self.arm.get_current_pose(self.arm_end_effector_link).pose			
			shift_x_pose=deepcopy(start_shaking_pose)
			shift_z_pose=deepcopy(start_shaking_pose)
			shift_x_pose.position.x+=incre_x
			shift_z_pose.position.z+=incre_z
			if cartesian: 
				plan_z,end_joint_state_z,signal_z=self.cts(start_shaking_pose,shift_z_pose,300)
				plan_x,end_joint_state_x,signal_x=self.cts(start_shaking_pose,shift_x_pose,300)		
			shaking_state=self.arm.get_current_joint_values()
			if (shake_times%4!=0 and signal_x) or (shake_times%4==0 and not signal_z):
				self.shaking(shaking_state,end_joint_state_x,freq,6)
			elif shake_times%4==0 and signal_z:
				self.shaking(shaking_state,end_joint_state_z,freq+1,5)
			rospy.loginfo("shaking times :  "+str(shake_times)+ " .")
			#ROS  subscribe to  "/color_area_ratio"
			area_ratio= rospy.wait_for_message('/color_area_ratio',Float64)
			rospy.loginfo("area ratio:  "+str(area_ratio.data)+ " .")
			shake_times+=1	
			if area_ratio.data<0.15  and shake_times==9:
				if  rotate<6:
					rotate+=1
					incre_x+=0.02
					incre_z+=0.01
					freq+=0.25
					shaking_state=self.arm.get_current_joint_values()
					shaking_state[6]-=0.1
					self.arm.set_joint_value_target(shaking_state)
					self.arm.go()
				rospy.loginfo("next round of shaking .")
				shake_times=0
			if area_ratio.data>0.15 or rotate==6:
				shaking_signal=False
		rospy.sleep(2)
		shaking_state=self.arm.get_current_joint_values()
		shaking_state[6]+=0.1*rotate+rotate_ori
		self.arm.set_joint_value_target(shaking_state)
		self.arm.go()
		rospy.sleep(3)
		self.arm.set_joint_value_target(start_trans_pose)
		self.arm.go()
		rospy.sleep(4)
		self.gripperCtrl(255)
		rospy.sleep(3)
		self.arm.set_joint_value_target(j_ori_state)
		self.arm.go()
		rospy.sleep(5)
					
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
		