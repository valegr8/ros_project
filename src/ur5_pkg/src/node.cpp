/**
 * @file node.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-03-18
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <ur5_pkg/transformation_matrix.h>
#include <ur5_pkg/kinematics.h>
#include <ur5_pkg/utils.h>

#define LOOP_RATE_FREQUENCY 0.5  /**< used to set run loops frequency*/

 /********************/
 /* GLOBAL VARIABLES */
 /********************/
vector<ros::Subscriber> subscribers(JOINT_NUM); /**< global subscribers vector*/
vector<long double> jointState(JOINT_NUM); /**< contains all /state values of the joints*/

 /********************/
 /* STATIC FUNCTIONS */
 /********************/
static void get_position_shoulder_pan(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {jointState[0] = ctr_msg->process_value;}
static void get_position_shoulder_lift(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {jointState[1] = ctr_msg->process_value;}
static void get_position_elbow(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {jointState[2] = ctr_msg->process_value;}
static void get_position_wrist_1(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {jointState[3] = ctr_msg->process_value;}
static void get_position_wrist_2(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {jointState[4] = ctr_msg->process_value;}
static void get_position_wrist_3(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {jointState[5] = ctr_msg->process_value;}

static void set_subscribers(ros::NodeHandle n)
{
    int queue_size = 1000;
    subscribers[0] = n.subscribe("/shoulder_pan_joint_position_controller/state", queue_size, get_position_shoulder_pan);
    subscribers[1] = n.subscribe("/shoulder_lift_joint_position_controller/state", queue_size, get_position_shoulder_lift);
    subscribers[2] = n.subscribe("/elbow_joint_position_controller/state", queue_size, get_position_elbow);
    subscribers[3] = n.subscribe("/wrist_1_joint_position_controller/state", queue_size, get_position_wrist_1);
    subscribers[4] = n.subscribe("/wrist_2_joint_position_controller/state", queue_size, get_position_wrist_2);
    subscribers[5] = n.subscribe("/wrist_3_joint_position_controller/state", queue_size, get_position_wrist_3);
}


int main (int argc, char **argv)
{
    cout << "Starting ROS\n";

    ros::init(argc, argv, "node");
    ros::NodeHandle nodeHandle;
    ros::Rate loop_rate(LOOP_RATE_FREQUENCY);

    set_subscribers(nodeHandle);
     
    //test_function
    while(ros::ok())
    {
        ros::spinOnce();

        print_joints(jointState);
        
        pair<Vector3ld, Matrix3ld> forward = computeForwardKinematics(jointState);
        print_position(forward.first(0), forward.first(1), forward.first(2));
        cout << BLUE << "Rotation matrix: " << NC << endl << forward.second << endl;

        Matrix86ld inversa = computeInverseKinematics(forward.first, forward.second);
        cout << BLUE << "Inverse Kinematic Matrix: " << NC << endl << inversa << endl;

        Matrix6ld jacobian = computeJacobian(jointState);
        cout << BLUE << "Jacobian matrix: " << NC << endl << jacobian << endl;

        cout << endl << MAGENTA << "----------------------------------------------------------" << NC << endl << endl;

        loop_rate.sleep();
        sleep(3);
    }

    ROS_ERROR("Ros not working\n");
    return 1; 
}