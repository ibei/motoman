#!/usr/bin/python

import roslib; roslib.load_manifest('motoman_driver')
import rospy
import sensor_msgs.msg
import trajectory_msgs.msg
import copy
import rospy,sys
import moveit_commander
import numpy as np
from std_msgs.msg import Duration
import moveit_commander
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
    		#0arm= 0
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
      			
	def add_target(self,f_target_pose,f_target_size,frame,x,y,o1,o2,o3,o4):
		f_target_pose.header.frame_id=frame
		f_target_pose.pose.position.x=x
		f_target_pose.pose.position.y=y
		f_target_pose.pose.position.z=self.table_ground+self.table_size[2]+f_target_size[2]/2.0
		f_target_pose.pose.orientation.x=o1
		f_target_pose.pose.orientation.y=o2
		f_target_pose.pose.orientation.z=o3
		f_target_pose.pose.orientation.w=o4
		#self.scene.add_box(f_target_id,f_target_pose,f_target_size)	
      			
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
			if  (attempts %maxtries==0 and fraction!=1):
				rospy.loginfo("path planning failed with  " + str(fraction*100)+"% success.")
				return 0,0,0
				continue
			elif  fraction==1:	
				rospy.loginfo("path compute successfully with "+str(attempts)+" attempts.")	
				#print(plan.joint_trajectory.points[-5].velocities)
				#for i in range(len(plan.joint_trajectory.points)):
					#plan.joint_trajectory.points[i].velocities=[0,0,0,0,0,0,0]
				#plan = self.arm.retime_trajectory(self.robot.get_current_state(), plan, 1.0)
				rospy.sleep(3)
				if exe_signal:
					q_traj=[self.arm.get_current_joint_values()]
					t_traj=[0.0]
					for i in range(2,len(plan.joint_trajectory.points)):
						q_traj.append(plan.joint_trajectory.points[i].positions)
						t_traj.append(t_traj[-1]+0.03)
					self.FollowQTraj(q_traj,t_traj)
					self.sub_jpc.publish(self.ToROSTrajectory(self.JointNames(), q_traj, t_traj, self.dq_traj))
					rospy.sleep(5)
					#self.sub_jpc.publish(plan)
					#self.arm.execute(plan)
				end_joint_state=plan.joint_trajectory.points[-1].positions	
				signal=1
				return plan,end_joint_state, signal
				
	#move and rotate
	def cts_rotate(self,start_pose,end_pose,angle,maxtries,exe_signal=False):
		waypoints=[]
		fraction=0.0
		attempts=0
		#maxtries_z=300
		waypoints.append(start_pose)	
		waypoints.append(end_pose)
		while fraction!=1 and attempts<maxtries:
			(plan,fraction)=self.arm.compute_cartesian_path(waypoints,0.005,0.0,True)
			attempts+=1
			if  (attempts %maxtries==0 and fraction!=1):
				rospy.loginfo("path planning failed with  " + str(fraction*100)+"% success.")
				return 0,0,0
				continue
			elif  fraction==1:	
				rospy.loginfo("path compute successfully with "+str(attempts)+" attempts.")	
				rospy.sleep(3)
				if exe_signal:
					q_traj=[self.arm.get_current_joint_values()]
					t_traj=[0.0]
					per_angle=angle/(len(plan.joint_trajectory.points)-2)
					for i in range(2,len(plan.joint_trajectory.points)):
						plan.joint_trajectory.points[i].positions[6]-=per_angle*(i-2)
						q_traj.append(plan.joint_trajectory.points[i].positions)
						t_traj.append(t_traj[-1]+0.03)
					self.FollowQTraj(q_traj,t_traj)
					self.sub_jpc.publish(self.ToROSTrajectory(self.JointNames(), q_traj, t_traj, self.dq_traj))
					rospy.sleep(5)
					#self.sub_jpc.publish(plan)
					#self.arm.execute(plan)
				end_joint_state=plan.joint_trajectory.points[-1].positions	
				signal=1
				return plan,end_joint_state, signal
	# shaking function: 
	# freq : shaking freqence
	# times : shaking time per action  
	def shaking(self,initial_state,start_joint_state,end_joint_state,freq,times):
		q_traj=[initial_state]
		t_traj=[0.0]
		for i in range(times):
			q_traj.append(end_joint_state)
			t_traj.append(t_traj[-1]+0.5/freq)
			q_traj.append(start_joint_state)
			t_traj.append(t_traj[-1]+0.5/freq)
		q_traj.append(initial_state)
		t_traj.append(t_traj[-1]+0.5/freq)
		self.FollowQTraj(q_traj,t_traj)
		self.sub_jpc.publish(self.ToROSTrajectory(self.JointNames(), q_traj, t_traj, self.dq_traj))
		rospy.sleep(6)
		
	def setupSence(self):	
		r_tool_size=[0.03,0.02,0.18]
		l_tool_size=[0.03,0.02,0.18]
		#real scene table
		table_pose=PoseStamped()
		table_pose.header.frame_id=self.reference_frame
		table_pose.pose.position.x=-0.0
		table_pose.pose.position.y=0.65
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
		
	def grasp_pose(self,pose,r,theta):
		g_p=PoseStamped()
		g_p.header.frame_id=self.arm_end_effector_link
		g_p.pose.position.x=pose.pose.position.x+r*math.sin(theta)
		g_p.pose.position.y=pose.pose.position.y-r*math.cos(theta)
		g_p.pose.position.z=pose.pose.position.z+0.05
		g_p.pose.orientation.w=0.5*(math.cos(0.5*theta)-math.sin(0.5*theta))
		g_p.pose.orientation.x=-0.5*(math.cos(0.5*theta)+math.sin(0.5*theta))
		g_p.pose.orientation.y=0.5*(math.cos(0.5*theta)-math.sin(0.5*theta))
		g_p.pose.orientation.z=0.5*(math.sin(0.5*theta)+math.cos(0.5*theta))
		return g_p
	
	def pour_pose(self,pose,h,r,theta):
		p_p=PoseStamped()
		p_p.header.frame_id=self.arm_end_effector_link
		p_p.pose.position.x=pose.pose.position.x-r*math.cos(theta)
		p_p.pose.position.y=pose.pose.position.y+r*math.sin(theta)
		p_p.pose.position.z=pose.pose.position.z+h
		theta*=-1
		p_p.pose.orientation.w=0.5*(math.cos(0.5*theta)-math.sin(0.5*theta))
		p_p.pose.orientation.x=-0.5*(math.cos(0.5*theta)+math.sin(0.5*theta))
		p_p.pose.orientation.y=0.5*(math.cos(0.5*theta)-math.sin(0.5*theta))
		p_p.pose.orientation.z=0.5*(math.sin(0.5*theta)+math.cos(0.5*theta))
		return p_p	
		
	def pour_rotate(self,angle_pre,angle_r,r,maxtries):
		initial_state=self.arm.get_current_joint_values()
		initial_pose=self.arm.get_current_pose(self.arm_end_effector_link)
		final_pose=deepcopy(initial_pose)
		final_pose.pose.position.x+=r*(1-math.cos(angle_r))*math.cos(angle_pre)
		final_pose.pose.position.y-=r*(1-math.cos(angle_r))*math.sin(angle_pre)
		final_pose.pose.position.z+=r*math.sin(angle_r)*math.cos(angle_r)
		plan,end_joint_state, signal=self.cts_rotate(initial_pose.pose,final_pose.pose,angle_r,maxtries,True)
		return plan,end_joint_state, signal
		
	def move_back(self,back_pose,maxtries):
		current_pose=self.arm.get_current_pose(self.arm_end_effector_link).pose
		plan, end_joint_state,signal=self.cts(current_pose,back_pose,maxtries,True)
		return signal

	#def get_to_target_pose(self,start_state,target_state) 	
	def __init__(self):
		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('moveit_demo')
		self.scene=PlanningSceneInterface()
		pub_traj= rospy.Publisher('/joint_path_command', trajectory_msgs.msg.JointTrajectory, queue_size=10)
		pub_ratio_sig= rospy.Publisher('/enable_source_change',Float64,queue_size=1)
		self.scene_pub=rospy.Publisher('planning_scene',PlanningScene)
		self.gripperCtrl=rospy.ServiceProxy("/two_finger/gripper/gotoPositionUntilTouch",SetPosition)
		#self.m2j=rospy.Publisher("/two_finger/motoman_control/move_to_joint",JointAnglesDuration,queue_size=1,latch=True)
		self.colors=dict()
		rospy.sleep(1)
		self.robot=moveit_commander.robot.RobotCommander()
		self.arm=MoveGroupCommander('arm')
		self.arm.allow_replanning(True)
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
		self.target1_id='target1_object'
		self.target2_id='target2_object'
		self.target3_id='target3_object'
		self.target4_id='target4_object'
		self.f_target_id='receive_container'
		self.scene.remove_world_object(self.l_id)
		self.scene.remove_world_object(self.r_id)
		self.scene.remove_world_object(self.table_id)
		self.scene.remove_world_object(self.target1_id)
		self.scene.remove_world_object(self.target2_id)
		self.scene.remove_world_object(self.target3_id)
		self.scene.remove_world_object(self.target4_id)
		#self.scene.remove_attached_object(self.arm_end_effector_link,self.target_id)
		self.scene.remove_world_object(self.f_target_id)

		self.table_ground=0.13
		self.table_size=[0.9,0.6,0.018]
		self.setupSence()
		target1_size=[0.035,0.035,0.19]
		target2_size=target1_size
		target3_size=target1_size
		target4_size=target1_size
		self.f_target_size=[0.2,0.2,0.04]
		
		f_target_pose=PoseStamped()
		pre_pour_pose=PoseStamped()
		target1_pose=PoseStamped()
		target2_pose=PoseStamped()
		target3_pose=PoseStamped()
		target4_pose=PoseStamped()
		
		joint_names= ['joint_'+jkey for jkey in ('s','l','e','u','r','b','t')]
   		joint_names= rospy.get_param('controller_joint_names')
		traj= trajectory_msgs.msg.JointTrajectory()
		traj.joint_names= joint_names
		
		#final target
		#self.add_target(f_target_pose,self.f_target_size,self.reference_frame,-0.184+0.27,0.62+0.1,0,0,0,1)
		self.add_target(f_target_pose,self.f_target_size,self.reference_frame,0.3,0.6,0,0,0,1)
		self.scene.add_box(self.f_target_id,f_target_pose,self.f_target_size)
		#self.add_target(pre_pour_pose,self.reference_frame,x,y,0,0,0,1)
		
		#target localization
		msg = rospy.wait_for_message('/aruco_single/pose',PoseStamped)	
		self.add_target(target1_pose,target1_size,self.reference_frame,-0.25,0.8,0,0,0,1)
		self.scene.add_box(self.target1_id,target1_pose,target1_size)
		self.add_target(target2_pose,target2_size,self.reference_frame,-0.12,0.87,0,0,0,1)
		self.scene.add_box(self.target2_id,target2_pose,target2_size)
		self.add_target(target3_pose,target3_size,self.reference_frame,0.02,0.88,0,0,0,1)
		self.scene.add_box(self.target3_id,target3_pose,target3_size)
		self.add_target(target4_pose,target4_size,self.reference_frame,0.12,0.81,0,0,0,1)
		self.scene.add_box(self.target4_id,target4_pose,target4_size)
		
		#pouring pose
		pour_pose=f_target_pose
		pour_pose.pose.position.x-=0.06
		pour_pose.pose.position.y-=0.12
		pour_pose.pose.position.z+=0.15
		pour_pose.pose.orientation.x=-0.5
		pour_pose.pose.orientation.y=-0.5
		pour_pose.pose.orientation.z=-0.5
		pour_pose.pose.orientation.w=0.5
		
		#attach_pose
		g_p=PoseStamped()
		g_p.header.frame_id=self.arm_end_effector_link
		g_p.pose.position.x=0.00
		g_p.pose.position.y=-0.00
		g_p.pose.position.z=0.025
		g_p.pose.orientation.w=0.707
		g_p.pose.orientation.x=0
		g_p.pose.orientation.y=-0.707
		g_p.pose.orientation.z=0
		
		#set color
		self.setColor(self.target1_id,0.8,0,0,1.0)
		self.setColor(self.target2_id,0.8,0,0,1.0)
		self.setColor(self.target3_id,0.8,0,0,1.0)
		self.setColor(self.target4_id,0.8,0,0,1.0)
		self.setColor(self.f_target_id,0.8,0.3,0,1.0)
		self.setColor('r_tool',0.8,0,0)
		self.setColor('l_tool',0.8,0,0)
		self.sendColors()
		self.gripperCtrl(255)
		rospy.sleep(3)
		self.arm.set_named_target("initial_arm")
		self.arm.go()
		rospy.sleep(5)
		
		#j_ori_state=[-1.899937629699707, -0.5684762597084045, 0.46537330746650696, 2.3229329586029053, -0.057941947132349014, -1.2867668867111206, 0.2628822326660156]
		#j_ori_state=[-2.161055326461792, -0.6802523136138916, -1.7733728885650635, -2.3315746784210205, -0.5292841196060181, 1.4411976337432861, -2.2327845096588135]
		j_ori_state=[-1.2628753185272217, -0.442996621131897, -0.1326361745595932, 2.333048105239868, -0.15598002076148987, -1.2167049646377563, 3.1414425373077393]
		
		#signal= True
		self.arm.set_joint_value_target(j_ori_state)
		self.arm.go()
		rospy.sleep(3)
		
		start_pose=self.arm.get_current_pose(self.arm_end_effector_link)
		
		#parameter setup
		tar_num=4
		maxtries=300
		
		#picking 
		#input : target pose
		#output : policy and execute
		#cts : output : plan, end_joint_state, signal
		#m=0
		#source pouring ratio list
		ratio_table=[, , , , ]
		for i in range(tar_num):
			#grasp target
			rospy.loginfo("Choosing source...")	
			if i==0:
				target_pose=target1_pose
				target_id=self.target1_id
				target_size=target1_size
			elif i==1:
				target_pose=target2_pose
				target_id=self.target2_id
				target_size=target2_size
			elif i==2:
				target_pose=target3_pose
				target_id=self.target3_id
				target_size=target3_size
			elif i==3:
				target_pose=target4_pose
				target_id=self.target4_id
				target_size=target4_size
			rospy.loginfo("Current variety of source: "+str(i+1))	
			#grasp :
			#-- if x>0: theta range ()
			p_i_radian=np.arctan(abs(target_pose.position.x/target_pose.position.y))
			p_i_angle=p_i_radian*180.0/3.14
			pick_list=[p_i_angle,5.0,25.0,45.0,65.0,15.0,35.0,55.0,75.0,10.0,20.0,30.0,40.0,50.0,60.0,70.0,80.0]
			for i in range(17):
				#m+=1
				#if target_pose.position.x>0: theta*=-1
				#if m%2==0: theta*=-1
				r=
				theta=(pick_list[i]/180.0)*3.14
				#start_pose=self.arm.get_current_pose(self.arm_end_effector_link).pose
				grasp_pose=self.grasp_pose(target_pose,r,theta)
				pre_g_pose=deepcopy(grasp_pose)
				#pick up
				if cartesian:
					plan_1, end_joint_state_1,signal_1=self.cts(start_pose.pose,grasp_pose.pose,maxtries,True)
					if signal_1==0: continue
					break
			rospy.sleep(2)
			self.gripperCtrl(0)
			rospy.sleep(2)
			
			#move to ori_pose
			signal_m_back=self.move_back(start_pose.pose)
			rospy.sleep(2)
			rospy.loginfo("Grasping done. Move to pour position.")	
			
			#move to target position
			r_pour=0.075
			h_pour=
			pour_angle=[0.0,15.0,30.0,45.0,60.0,75.0]
			
			for i in range(6):
				theta=(pour_angle[i]/180.0)*3.14
				pour_pose=self.pour_pose(f_target_pose,h_pour,r_pour,theta)
				#move to pose
				if cartesian:
					plan_1, end_joint_state_1,signal_1=self.cts_rotate(start_pose.pose,pour_pose.pose,1.57,maxtries,True)
					if signal_1==0: continue
					pre_pour_angle=pour_angle[i]
					break
			rospy.loginfo("Ready for pouring.")	
			rospy.sleep(2)
			
			#rotating and shaking
			#parameter: amp,freq,rotation_angle,
			#pouring pose (might be considered )
			#parameter setup
			#function: pour_rotate....for angle rotation before shaking
			#params: pouring pose angle with tgt, rotation_angle for pouring, bottle_radius, cts maxtries
			if i==0:
				amp=
				freq=
				r_angle=
			elif i==1:
				amp=
				freq=
				r_angle=
			elif i==2:
				amp=
				freq=
				r_angle=
			elif i==3:
				amp=
				freq=
				r_angle=
			'''
			pour_state=self.arm.get_current_joint_values()
			r_ori_angle=0.174           #10 degree
			#r_angle=r_ori_angle*
			pour_state[6]-=r_angle
			self.arm.set_joint_value_target(pour_state)
			self.arm.go()
			'''
			plan_p, end_joint_state_p,signal_p=self.pour_rotate(pre_pour_angle,r_angle,r,maxtries)
			if signal_p==0: 
				rospy.loginfo("pouring plan no success")
				break
			#shaking
			rospy.loginfo("Shaking planning...")
			shake_per_times=3
			shake_times=0
			shake_times_tgt=20
			signal=True
			rospy.loginfo("Parameter loaded")	
			rospy.sleep(1)
			start_shake_pose=self.arm.get_current_pose(self.arm_end_effector_link).pose# for trajectory of shaking
			start_joint_state=self.arm.get_current_joint_values() # for state[6] to rotate
			shift_pose=deepcopy(start_shake_pose)
			shift_pose.position.x+=amp*math.sin(r_angle)*math.cos(pre_pour_angle)# in verticle direction
			shift_pose.position.y-=amp*math.sin(r_angle)*math.sin(pre_pour_angle)
			shift_pose.position.z+=amp*math.cos(r_angle)#...
			if cartesian: 
				plan,end_joint_state,signal=self.cts(start_shake_pose,shift_pose,300)
			#shaking process
			while signal:
				area_ratio= rospy.wait_for_message('/color_area_ratio',Float64)
				if (area_ratio>ratio_table[i]) or (shake_times>shake_times_tgt): 
					pub_ratio_sig(1)
					break
				elif:
					self.shaking(start_joint_state,end_joint_state,freq,shake_per_times)
				shake_times+=1
				rospy.loginfo("shaking times :  "+str(shake_times)+ " .")
			continue
			
		#remove and shut down
		self.scene.remove_attached_object(self.arm_end_effector_link,'l_tool')
		self.scene.remove_attached_object(self.arm_end_effector_link,'r_tool')
		moveit_commander.roscpp_shutdown()
		moveit_commander.os._exit(0)
		
		
if __name__=="__main__":
	try:
			PickAndPlace()
	except KeyboardInterrupt:
			raise
		
