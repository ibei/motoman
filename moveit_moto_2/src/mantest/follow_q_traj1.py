#!/usr/bin/python
#\file    follow_q_traj1.py
#\brief   Following joint angle trajectory.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Nov.10, 2017
#src: motoman/motoman_driver/src/move_to_joint.py

import roslib; roslib.load_manifest('motoman_driver')
import rospy
import sensor_msgs.msg
import moveit_commander
import trajectory_msgs.msg
import copy
            

#Wait for subscribers (src: motoman_driver/move_to_joint.py)
def WaitForSubscribers(pub, timeout, num_subs=1):
  time_end= rospy.Time.now()+rospy.Duration(timeout)
  rate= rospy.Rate(10)
  while all((pub.get_num_connections()<num_subs, rospy.Time.now()<time_end, not rospy.is_shutdown())):
    rate.sleep()
  return (pub.get_num_connections()>=num_subs)


if __name__=='__main__':
    rospy.init_node('motoman_test')

    pub_traj= rospy.Publisher('/joint_path_command', trajectory_msgs.msg.JointTrajectory, queue_size=1)
    if not WaitForSubscribers(pub_traj, 3.0):
        print 'WARNING: No subscribers of /joint_path_command'

    joint_names= ['joint_'+jkey for jkey in ('s','l','e','u','r','b','t')]
    joint_names= rospy.get_param('controller_joint_names')
    
    
    
    #arm=moveit_commander.MoveGroupCommander("arm")
    #joint_positions=  [0.113, 1.548, 1.767, 0.535, 0, -1.168, -0.197]
    #joint_positions=  [0.113, 1.548, 1.767, 0.535, 0, -1.168, -0.197]
    #arm.set_joint_value_target(joint_positions)
    #arm.go()

    traj= trajectory_msgs.msg.JointTrajectory()
    traj.joint_names= joint_names

    def add_point(traj, time, positions, velocities=None):
        point= trajectory_msgs.msg.JointTrajectoryPoint()
        point.positions= copy.deepcopy(positions)
        if velocities is not None:
            point.velocities= copy.deepcopy(velocities)
            point.time_from_start= rospy.Duration(time)
            traj.points.append(point)


    add_point(traj, 0.3, [0.0]*7, [0.0]*7)
    add_point(traj, 0.3, [0, 0, 0, 0, 0,0.5, 0], [0.0]*7)
    add_point(traj, 0.3, [0, 0, 0, 0, 0,0, 0], [0.0]*7)
    add_point(traj, 0.3, [0, 0, 0, 0, 0,0.5, 0], [0.0]*7)
    add_point(traj, 0.3, [0, 0, 0, 0, 0,0, 0], [0.0]*7)
    add_point(traj, 0.3, [0, 0, 0, 0, 0,0.5, 0], [0.0]*7)
    add_point(traj, 0.3, [0, 0, 0, 0, 0,0, 0], [0.0]*7)
 
   #No specification of velocity (does not work!)
  #  WARNING: Velocity specification is mandatory.  If omitted, there is an error:
  #    Validation failed: Missing velocity data for trajectory pt 1
  
    traj.header.stamp= rospy.Time.now()

    pub_traj.publish(traj)

    rospy.signal_shutdown('Done.')
