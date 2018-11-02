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
      			
	def add_target(self,target_pose,target_size,frame,x,y,o1,o2,o3,o4):
		target_pose.header.frame_id=frame
		target_pose.pose.position.x=x
		target_pose.pose.position.y=y
		target_pose.pose.position.z=self.table_ground+self.table_size[2]+target_size[2]/2.0
		target_pose.pose.orientation.x=o1
		target_pose.pose.orientation.y=o2
		target_pose.pose.orientation.z=o3
		target_pose.pose.orientation.w=o4
		#self.scene.add_box(f_target_id,f_target_pose,f_target_size)	
      			
	def cts(self,start_pose,mid_pose,end_pose,maxtries,exe_signal=False):
		waypoints=[]
		fraction=0.0
		attempts=0
		waypoints.append(start_pose.pose)	
		if mid_pose!=0:
			waypoints.append(mid_pose.pose)
		waypoints.append(end_pose.pose)
		while fraction!=1 and attempts<maxtries:
			(plan,fraction)=self.arm.compute_cartesian_path(waypoints,0.005,0.0,True)
			attempts+=1
			if  (attempts %maxtries==0 and fraction!=1):
				rospy.loginfo("path planning failed with  " + str(fraction*100)+"% success.")
				return 0,0
				continue
			elif  fraction==1:	
				rospy.loginfo("path compute successfully with "+str(attempts)+" attempts.")	
				if exe_signal:
					q_traj=[self.arm.get_current_joint_values()]
					t_traj=[0.0]
					for i in range(2,len(plan.joint_trajectory.points)):
						q_traj.append(plan.joint_trajectory.points[i].positions)
						t_traj.append(t_traj[-1]+0.07)
					self.FollowQTraj(q_traj,t_traj)
					self.sub_jpc.publish(self.ToROSTrajectory(self.JointNames(), q_traj, t_traj, self.dq_traj))
					rospy.sleep(5)
				end_joint_state=plan.joint_trajectory.points[-1].positions	
				signal=1 
				return signal,end_joint_state
	
				
	#move and rotate
	def cts_rotate(self,start_pose,end_pose,angle_r,maxtries,exe_signal=False):
		angle=angle_r*3.14/180.0
		waypoints=[]
		fraction=0.0
		attempts=0
		waypoints.append(start_pose.pose)	
		waypoints.append(end_pose.pose)
		while fraction!=1 and attempts<maxtries:
			(plan,fraction)=self.arm.compute_cartesian_path(waypoints,0.005,0.0,True)
			attempts+=1
			if  (attempts %maxtries==0 and fraction!=1):
				rospy.loginfo("path planning failed with  " + str(fraction*100)+"% success.")
				return 0,0.0
				continue
			elif  fraction==1:	
				rospy.loginfo("path compute successfully with "+str(attempts)+" attempts.")	
				if exe_signal:
					q_traj=[self.arm.get_current_joint_values()]
					t_traj=[0.0]
					per_angle=angle/(len(plan.joint_trajectory.points)-2)
					for i in range(2,len(plan.joint_trajectory.points)):
						joint_inc_list = [j for j in plan.joint_trajectory.points[i].positions]
						#plan.joint_trajectory.points[i].positions[6]-=per_angle*(i-1)
						joint_inc_list[6]-=per_angle*(i-1)
						q_traj.append(joint_inc_list)
						t_traj.append(t_traj[-1]+0.05)
					self.FollowQTraj(q_traj,t_traj)
					self.sub_jpc.publish(self.ToROSTrajectory(self.JointNames(), q_traj, t_traj, self.dq_traj))
					rospy.sleep(5)
				end_joint_state=plan.joint_trajectory.points[-1].positions	
				signal=1
				return  signal,end_joint_state
				
	# shaking function: 
	# freq : shaking freqence
	# times : shaking time per action  
	def shaking(self,initial_state,end_joint_state,freq,times):
		q_traj=[initial_state]
		t_traj=[0.0]
		for i in range(times):
			q_traj.append(end_joint_state)
			t_traj.append(t_traj[-1]+0.5/freq)
			q_traj.append(initial_state)
			t_traj.append(t_traj[-1]+0.5/freq)
		self.FollowQTraj(q_traj,t_traj)
		self.sub_jpc.publish(self.ToROSTrajectory(self.JointNames(), q_traj, t_traj, self.dq_traj))
		rospy.sleep(6)
		
	#shaking_a: vertical bottle axis
	def shake_a(self,pre_p_angle,r_angle,amp):
		start_shake_pose=self.arm.get_current_pose(self.arm_end_effector_link)# for trajectory of shaking
		start_joint_state=self.arm.get_current_joint_values() # for state[6] to rotate
		shift_pose=deepcopy(start_shake_pose)
		r_angle=(r_angle/180.0)*3.14
		pre_p_angle=(pre_p_angle/180.0)*3.14
		shift_pose.pose.position.x+=amp*math.sin(r_angle)*math.cos(pre_p_angle)# in verticle direction
		shift_pose.pose.position.y-=amp*math.sin(r_angle)*math.sin(pre_p_angle)
		shift_pose.pose.position.z+=amp*math.cos(r_angle)#...
		signal,end_joint_state=self.cts(start_shake_pose,shift_pose,300)	
		return signal,end_joint_state
		
	def shake_b(self,pre_p_angle,r_angle,amp):
		start_shake_pose=self.arm.get_current_pose(self.arm_end_effector_link)# for trajectory of shaking
		start_joint_state=self.arm.get_current_joint_values() # for state[6] to rotate
		shift_pose=deepcopy(start_shake_pose)
		r_angle=(r_angle/180.0)*3.14
		pre_p_angle=(pre_p_angle/180.0)*3.14
		shift_pose.pose.position.x-=amp*math.cos(r_angle)*math.cos(pre_p_angle)# in verticle direction
		shift_pose.pose.position.y+=amp*math.cos(r_angle)*math.sin(pre_p_angle)
		shift_pose.pose.position.z+=amp*math.sin(r_angle)#...
		signal,end_joint_state=self.cts(start_shake_pose,shift_pose,300)	
		return signal,end_joint_state
				
		
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
		
	#Params: pose of bottle, grasp_radius, grasp_height, grasp_theta
	def get_grasp_pose(self,pose,r,theta):
		g_p=PoseStamped()
		g_p.header.frame_id=self.arm_end_effector_link
		g_p.pose.position.x=pose.pose.position.x+r*math.sin(theta)
		g_p.pose.position.y=pose.pose.position.y-r*math.cos(theta)
		g_p.pose.position.z=pose.pose.position.z
		g_p.pose.orientation.w=0.5*(math.cos(0.5*theta)-math.sin(0.5*theta))
		g_p.pose.orientation.x=-0.5*(math.cos(0.5*theta)+math.sin(0.5*theta))
		g_p.pose.orientation.y=0.5*(math.cos(0.5*theta)-math.sin(0.5*theta))
		g_p.pose.orientation.z=0.5*(math.sin(0.5*theta)+math.cos(0.5*theta))
		return g_p
	
	def get_pour_pose(self,pose,h,r,theta):
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
		angle_pre=(angle_pre/180.0)*3.14
		angle_r_1=(angle_r/180.0)*3.14
		initial_state=self.arm.get_current_joint_values()
		initial_pose=self.arm.get_current_pose(self.arm_end_effector_link)
		final_pose=deepcopy(initial_pose)
		final_pose.pose.position.x+=r*(1-math.cos(angle_r_1))*math.cos(angle_pre)
		final_pose.pose.position.y+=r*(1-math.cos(angle_r_1))*math.sin(angle_pre)
		final_pose.pose.position.z+=r*math.sin(angle_r_1)
		signal,e_j_s=self.cts_rotate(initial_pose,final_pose,angle_r,maxtries,True)
		return signal
		


	def pg_g_pp(self,pose,r,pre_r):
		start_pose=self.arm.get_current_pose(self.arm_end_effector_link)
		p_i_radian=np.arctan(abs(pose.pose.position.x/pose.pose.position.y))
		p_i_angle=p_i_radian*180.0/3.14
		pick_list=[0.0,p_i_angle,5.0,25.0,45.0,65.0,15.0,35.0,55.0,75.0,10.0,20.0,30.0,40.0,50.0,60.0]
		
		for i in range(len(pick_list)):
			theta=(pick_list[i]/180.0)*3.14
			if pose.pose.position.x>0:
				theta*=-1.0
			pre_grasp_pose=self.get_grasp_pose(pose,r+pre_r,theta)
			#pick up
			grasp_pose=pre_grasp_pose
			grasp_pose.pose.position.x-=pre_r*math.sin(theta)
			grasp_pose.pose.position.y+=pre_r*math.cos(theta)
			signal,e_j_s=self.cts(start_pose,pre_grasp_pose,grasp_pose,300,True)
			if signal==0: continue
			break
		rospy.sleep(1)
		self.gripperCtrl(0)
		rospy.sleep(2)
		#move to ori_pose
		current_pose=self.arm.get_current_pose(self.arm_end_effector_link)
		signal,e_j_s=self.cts(current_pose,pre_grasp_pose,start_pose,300,True)
		rospy.sleep(2)
		rospy.loginfo("Grasping done.")
		return start_pose,pre_grasp_pose,grasp_pose	
		
	def pp_ps(self,target_pose,pour_angle,r_pour,h_pour):
		for i in range(len(pour_angle)):
			maxtries=300
			start_pose=self.arm.get_current_pose(self.arm_end_effector_link)
			theta=(pour_angle[i]/180.0)*3.14
			pour_pose=self.get_pour_pose(target_pose,h_pour,r_pour,theta)
			#move to pose
			signal_1,e_j_s=self.cts_rotate(start_pose,pour_pose,90.0,maxtries,True)
			if signal_1==0: continue
			pre_pour_angle=pour_angle[i]
			rospy.loginfo("Ready for pouring.")	
			return pre_pour_angle
		
	def go_back(self,ori_pose,pre_grasp_pose,grasp_pose):
		cur_pose=self.arm.get_current_pose(self.arm_end_effector_link)
		signal,e1=self.cts(cur_pose,0,ori_pose,300,True)
		rospy.loginfo("back to pre_grasp pose, ready for placing bottle..")
		signal_1,e2=self.cts(ori_pose,pre_grasp_pose,grasp_pose,300,True)
		rospy.loginfo("back to grasp pose, open gripper..")
		self.gripperCtrl(255)
		rospy.sleep(1)
		signal_2,e3=self.cts(grasp_pose,pre_grasp_pose,ori_pose,300,True)
		rospy.loginfo("back to pre_grasp pose, ready for next kind of source.")
	def rotate_back(self,angle):
		angle=angle*3.14/180.0
		current_j_state=self.arm.get_current_joint_values()
		current_j_state[6]+=angle+1.57
		self.arm.go()
		
	def last_step_go_back(self,ori_pose):
		cur_pose=self.arm.get_current_pose(self.arm_end_effector_link)
		signal,e1=self.cts(cur_pose,0,ori_pose,300,True)
		rospy.loginfo("back to last step...")
		rospy.sleep(1)
		
	 	
	def __init__(self):
		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('moveit_demo')
		self.scene=PlanningSceneInterface()
		pub_traj= rospy.Publisher('/joint_path_command', trajectory_msgs.msg.JointTrajectory, queue_size=10)
		#pub_ratio_sig= rospy.Publisher('/enable_source_change',Int64,queue_size=1)
		self.scene_pub=rospy.Publisher('planning_scene',PlanningScene)
		self.gripperCtrl=rospy.ServiceProxy("/two_finger/gripper/gotoPositionUntilTouch",SetPosition)
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
		self.scene.remove_world_object(self.f_target_id)
		#self.scene.remove_attached_object(self.arm_end_effector_link,self.target_id)

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
		self.add_target(f_target_pose,self.f_target_size,self.reference_frame,0.2,0.6,0,0,0,1)
		self.scene.add_box(self.f_target_id,f_target_pose,self.f_target_size)
		#self.add_target(pre_pour_pose,self.reference_frame,x,y,0,0,0,1)
		
		#target localization
		#msg = rospy.wait_for_message('/aruco_single/pose',PoseStamped)	
		self.add_target(target1_pose,target1_size,self.reference_frame,-0.25,0.8,0,0,0,1)
		self.scene.add_box(self.target1_id,target1_pose,target1_size)
		self.add_target(target2_pose,target2_size,self.reference_frame,-0.12,0.87,0,0,0,1)
		self.scene.add_box(self.target2_id,target2_pose,target2_size)
		self.add_target(target3_pose,target3_size,self.reference_frame,0.02,0.88,0,0,0,1)
		self.scene.add_box(self.target3_id,target3_pose,target3_size)
		self.add_target(target4_pose,target4_size,self.reference_frame,0.15,0.80,0,0,0,1)
		self.scene.add_box(self.target4_id,target4_pose,target4_size)
		
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
		rospy.sleep(3)
		
		#j_ori_state=[-1.899937629699707, -0.5684762597084045, 0.46537330746650696, 2.3229329586029053, -0.057941947132349014, -1.2867668867111206, 0.2628822326660156]
		#j_ori_state=[-2.161055326461792, -0.6802523136138916, -1.7733728885650635, -2.3315746784210205, -0.5292841196060181, 1.4411976337432861, -2.2327845096588135]
		j_ori_state=[-1.2628753185272217, -0.442996621131897, -0.1326361745595932, 2.333048105239868, -0.15598002076148987, -1.2167049646377563, 3.1414425373077393]
		
		#signal= True
		self.arm.set_joint_value_target(j_ori_state)
		self.arm.go()
		rospy.sleep(3)		
		#parameter setup
		tar_num=4
		maxtries=300
		r_grasp=0.15
		r_pre_g=0.1
		r_bottle=0.1
		for i in range(0,tar_num):
			#grasp target
			rospy.loginfo("Choosing source...")	
			if i==0:
				target_pose=target1_pose
				target_id=self.target1_id
				target_size=target1_size
				amp=0.1
				freq=2.75
				r_angle=45.0
			elif i==1:
				target_pose=target2_pose
				target_id=self.target2_id
				target_size=target2_size
				amp=0.15
				freq=2.0
				r_angle=30.0
			elif i==2:
				target_pose=target3_pose
				target_id=self.target3_id
				target_size=target3_size
				amp=0.1
				freq=2.75
				r_angle=45.0
			elif i==3:
				target_pose=target4_pose
				target_id=self.target4_id
				target_size=target4_size
				amp=0.1
				freq=2.75
				r_angle=45.0
				
			r_pour=0.1
			h_pour=0.1
			
			#pour_angle=[15.0,30.0,45.0,60.0,75.0]
			pour_angle=[0.0,-15.0,-10.0,-5.0,0.0,10.0,15.0,30.0,45.0,60.0,75.0]
			rospy.loginfo("Params loaded.")	
			rospy.loginfo("Current Source: "+str(i+1)+"  ")	
			#grasp and back
			ori_pose,mid_pose,g_pose=self.pg_g_pp(target_pose,r_grasp,r_pre_g)
			#move to target position for pouring
			pre_p_angle=self.pp_ps(f_target_pose,pour_angle,r_pour,h_pour)
			rospy.loginfo("pre_pour_angle : "+str(pre_p_angle))
			#rotation and shaking
			ps_signal=self.pour_rotate(pre_p_angle,r_angle,r_bottle,maxtries)
			if ps_signal !=1:
				 rospy.loginfo("Pre_shaking_rotation failed,  back to ori_pose...")
				 self.last_step_go_back(ori_pose)
				 rospy.sleep(3)
				 continue
			else:
				rospy.loginfo("Shaking planning...")
				shake_per_times=3
				shake_times=0
				shake_times_tgt=1
				signal=True
				rospy.loginfo("Parameter loaded")	
				rospy.sleep(1)
				signal,end_joint_state=self.shake_a(pre_p_angle,r_angle,amp)
				while signal==1:
					#area_ratio= rospy.wait_for_message('/color_area_ratio',Float64)
					#if (area_ratio>ratio_table[i]) or (shake_times>shake_times_tgt):
					if (shake_times<shake_times_tgt):
						self.shaking(start_joint_state,end_joint_state,freq,shake_per_times)
						shake_times+=1
						
						rospy.loginfo("shaking times :  "+str(shake_times)+ " .")
					else:
						signal=0
						
			self.rotate_back(r_angle)	
			self.go_back(ori_pose,mid_pose,g_pose)
			rospy.sleep(2)
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
		