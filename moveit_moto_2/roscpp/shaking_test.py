#! /usr/bin/env python

from cubic_hermite_spline import TCubicHermiteSpline
#from cubic_hermite_spline import *
import geometry_msgs.msg
import trajectory_msgs.msg
import rospy
import roslib
import copy
import numpy as np
import numpy.linalg as la
import math
import random
from motoman_control.msg import JointAnglesDuration,JointAngles



class TShakeTest(object):
  def __init__(self):
    self.joint_names= [[]]
    #joint_names= ['joint_'+jkey for jkey in ('s','l','e','u','r','b','t')]    ,queue_size=1
    self.joint_names[0]= rospy.get_param('controller_joint_names')
    self.traj= trajectory_msgs.msg.JointTrajectory()
    self.sub_jpc= rospy.Publisher('/joint_path_command', trajectory_msgs.msg.JointTrajectory,queue_size=10)
    self.m2j=rospy.Publisher("/two_finger/motoman_control/move_to_joint",JointAnglesDuration,queue_size=1,latch=True)
    #self.amp=0.3
    self.freq=2.0
    #axis=[0,0,0,0,0,1,0]
   
    x=[0,0,0,0,0,0,0]
    q_traj=[x]
    t_traj=[0.0]
    #place=JointAnglesDuration(JointAngles(-1.8147335052490234, -0.8826525807380676, 0.04519927501678467, 1.9301730394363403, -2.9218080043792725, 1.456112265586853, 1.5086852312088013),rospy.Duration(4))
    '''
    for i in range(3):
	q_traj.append([0,0,0,0.1,0.1,0.1,0])
	t_traj.append(t_traj[-1]+0.5/self.freq)
	q_traj.append([0,0,0,-0.1,-0.1,-0.1,0])
	t_traj.append(t_traj[-1]+0.5/self.freq)
    '''
    q_traj.append([-1.215628743171692, 0.22259767353534698, 3.0375857022590935e-05, 0.027216767892241478, -3.141425371170044, -0.19530156254768372, 1.92607843875885])
    t_traj.append(t_traj[-1]+2)
    #q_traj.append([0,0,0,0.1,0.2,0.4,0])
    #t_traj.append(t_traj[-1]+1)
    #q_traj.append([0,0,0,0.2,0.3,0.6,0])
    #t_traj.append(t_traj[-1]+1)
    #q_traj.append(x)
    #t_traj.append(t_traj[-1]+0.5/self.freq)
    self.SmoothQTraj(q_traj)
    self.FollowQTraj(q_traj,t_traj)
    #self.m2j.publish(place)
    self.sub_jpc.publish(self.ToROSTrajectory(self.JointNames(), q_traj, t_traj, self.dq_traj))

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
    print dq_traj
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
    print self.traj
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


if __name__=='__main__':

  rospy.init_node('shaking')
  robot= TShakeTest()
  #rospy.spin()





