### Explanation of the launch files

dummy_moto.launch: run python script to make a dummy robot for the rviz visualizer
rviz_traj_sim.launch: simulate the robot in rviz. (includes dummy_moto.launch)
motomansia10f_cmn.launch: start process to simulate robot with rviz. (rviz and dummy_moto nodes not included)
motomansia10f_real.launch: control real robot. (includes motomansia10f_cmn.launch)
rviz_n_control.launch: simulate robot and control real robot at the same time. (includes rviz_traj_sim.launch)