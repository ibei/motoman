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
import trajectory_msgs.msg
import copy
from std_msgs.msg  import Float32MultiArray
            

#Wait for subscribers (src: motoman_driver/move_to_joint.py)
def WaitForSubscribers(pub, timeout, num_subs=1):
  time_end= rospy.Time.now()+rospy.Duration(timeout)
  rate= rospy.Rate(10)
  while all((pub.get_num_connections()<num_subs, rospy.Time.now()<time_end, not rospy.is_shutdown())):
    rate.sleep()
  return (pub.get_num_connections()>=num_subs)
  
  
def PositionCmdCallback(msg):
    rospy.loginfo("%f %f %f",msg.data[0],msg.data[1],msg.data[2])
    global p1,p2,p3
    p1=msg.data[0]
    p2=msg.data[1]
    p3=msg.data[2]
    
    
    
if __name__=='__main__':
    rospy.init_node('motoman_test')

    pub_traj= rospy.Publisher('/joint_path_command', trajectory_msgs.msg.JointTrajectory, queue_size=1)
    sub_posi=rospy.Subscriber('chatter', Float32MultiArray, PositionCmdCallback)
    
    scene=PlanningSceneInterface()
	#scene_pub=rospy.Publisher('planning_scene',PlanningScene)
	#colors=dict()
	#rospy.sleep(1)
	arm=MoveGroupCommander('arm')
	end_effector_link=arm.get_end_effector_link()
	arm.set_goal_position_tolerance(0.01)
	arm.set_goal_orientation_tolerance(0.05)
	arm.allow_replanning(True)
		
	reference_frame='base'
		
	arm.set_pose_reference_frame(reference_frame)
		
	arm.set_planning_time(5)
		
	table_id='table'
		#box1_id='box1'
	#	box2_id='box2'
		
		#scene.remove_world_object(box1_id)
		#scene.remove_world_object(box2_id)
		scene.remove_world_object(table_id)
		
	rospy.sleep(1)
		
		
		#arm.set_named_target("resting")
		#arm.go()
	ros.sleep(2)
		
		
	table_ground=0.75
	table_size=[0.2,0.7,0.01]
		#box1_size=[0.1,0.05,0.05]
		#box2_size=[0.05,0.05,0.15]
		
		
	table_pose=PostStamped()
	table_pose.header.frame_id=reference_frame
	table_pose.pose.position.x=0.26
	table_pose.pose.position.y=0.0
	table_pose.pose.position.z=table_ground+table_size[2]/2.0
	table_pose.pose.orientation.w=1.0
	scene.add_box(table_id,table_pose,table_size)
    
     
    if not WaitForSubscribers(pub_traj, 3.0):
        print 'WARNING: No subscribers of /joint_path_command'

    joint_names= ['joint_'+jkey for jkey in ('s','l','e','u','r','b','t')]
    joint_names= rospy.get_param('controller_joint_names')

    traj= trajectory_msgs.msg.JointTrajectory()
    traj.joint_names= joint_names

    def add_point(traj, time, positions, velocities=None):
        point= trajectory_msgs.msg.JointTrajectoryPoint()
        point.positions= copy.deepcopy(positions)
        if velocities is not None:
            point.velocities= copy.deepcopy(velocities)
            point.time_from_start= rospy.Duration(time)
            traj.points.append(point)
            print (traj.points)


    add_point(traj, 5.0, [0.0]*7, [0.0]*7)
  #  add_point(traj, 10.0, [p1]*7, [0.0]*7)
    add_point(traj, 15.0, [0,1.57,0,-1.57,0,1.57,1], [0.0]*7)
   #add_point(traj, 15.0, [0,3.14,0,3.14,0,3.14,0], [0.0]*7)
   # add_point(traj, 8.0, [1.50593, 1.29833, 1.55332, 1.04798, 0, 0.460345, 1.29779], [0.0]*7)
  #  add_point(traj, 10.0, [1.61328, 1.33451, 1.68876, 1.68903, 0, 0.797354, 1.33417], [0.0]*7)
    #add_point(traj, 12.0, [-0.271554, 1.7564, -0.985498, 1.88689, 0, -1.55927, -0.96007], [0.0]*7)
   # add_point(traj, 14.0, [0, 0.157985, 0, -2.05464, 0, 0.641824, 0], [0.0]*7)
  #add_point(traj, 12.0, [-0.271554, 1.7564, -0.985498, 1.88689, 0, -1.55927, -0.96007], [0.0]*7)
  #add_point(traj, 14.0, [0, 0.157985, 0, -2.05464, 0, 0.641824, 0], [0.0]*7)
  #   add_point(traj, 16.0, [0.0]*7, [0.0]*7)
 # add_point(traj, 6.0, [0.1,  -0.3,  0.15, -0.7,  0.1,  -0.3,  0.15], [0.0]*7)  #Zero velocity
  #add_point(traj, 6.0, [0.1,  -0.3,  0.15, -0.7,  0.1,  -0.3,  0.15])  #No specification of velocity (does not work!)
  #  WARNING: Velocity specification is mandatory.  If omitted, there is an error:
  #    Validation failed: Missing velocity data for trajectory pt 1
  


    traj.header.stamp= rospy.Time.now()

    pub_traj.publish(traj)

    rospy.signal_shutdown('Done.')
