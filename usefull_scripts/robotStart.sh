catkin build --env-cache 
source devel/setup.bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/rosProg/ros_project/src/ur5_pkg/world/files
roslaunch ur5_pkg ur5_gripper.launch
