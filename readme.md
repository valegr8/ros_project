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

Add to .bashrc:

	export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:{$WORK_DIR}/ros_project/src/ur5_pkg/models
	
	
Install python dependences:

	pip3 install torch torchvision
	sudo apt install python3-pandas
	sudo apt install python3-tqdm
	pip install seaborn
	
