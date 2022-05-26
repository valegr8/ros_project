To launch Gazebo:

	catkin_make OPPURE catkin build --env-cache
	source devel/setup.bash
	roslaunch ur5_pkg ur5_gripper.launch

Fast all-in-one command:

	catkin build --env-cache && source devel/setup.bash && clear && roslaunch ur5_pkg ur5_gripper.launch

To run our node in a new shell:

	source devel/setup.bash	
	rosrun ur5_pkg node

Fast all-in-one command:

	catkin build --env-cache && source devel/setup.bash && clear && rosrun ur5_pkg node
	source devel/setup.bash && clear && rosrun ur5_pkg node

To launch the dynamic link plugin:

	catkin build --env-cache && source devel/setup.bash && clear && roslaunch gazebo_ros_link_attacher test_attacher.launch
	
	source devel/setup.bash 
	rosrun gazebo_ros_link_attacher spawn_models.py
	rosrun gazebo_ros_link_attacher attach.py
	rosrun gazebo_ros_link_attacher detach.py
