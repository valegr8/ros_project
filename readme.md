Per far partire il programma:

catkin_make


source devel/setup.bash

roslaunch ur5_pkg ur5_gripper.launch

in un secondo terminale:

source devel/setup.bash
rosrun ur5_pkg node

per lanciare il nodo

aggiunto service forward kinematic
