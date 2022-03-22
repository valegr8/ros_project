Per far partire Gazebo ed il robot:

	catkin_make OPPURE catkin build --env-cache
	source devel/setup.bash
	roslaunch ur5_pkg ur5_gripper.launch

Per avviare il file cpp:

	source devel/setup.bash	
	rosrun ur5_pkg node

P.S. Usare catkin build è preferibile
