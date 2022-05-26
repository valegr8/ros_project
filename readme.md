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

To use the dynamic-link plugin:

Attaccare:
rosservice call /link_attacher_node/attach "model_name_1: 'cube1'
link_name_1: 'link'
model_name_2: 'cube2'
link_name_2: 'link'"

Staccare:
rosservice call /link_attacher_node/detach "model_name_1: 'cube1'
link_name_1: 'link'
model_name_2: 'cube2'
link_name_2: 'link'"
