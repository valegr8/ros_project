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
#include <std_msgs/Float64.h>

#define LOOP_RATE_FREQUENCY 0.5  /**< used to set run loops frequency*/

 /********************/
 /* GLOBAL VARIABLES */
 /********************/
static vector<ros::Subscriber> subscribers(JOINT_NUM); /**< global subscribers vector*/
static vector<ros::Publisher> publishers(JOINT_NUM);  /**< global publisher vector*/
static vector<long double> jointState(JOINT_NUM); /**< contains all /state values of the joints*/
static int queue_size; /**< used for publisher and subscribers queue size */


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
    subscribers[0] = n.subscribe("/shoulder_pan_joint_position_controller/state", queue_size, get_position_shoulder_pan);
    subscribers[1] = n.subscribe("/shoulder_lift_joint_position_controller/state", queue_size, get_position_shoulder_lift);
    subscribers[2] = n.subscribe("/elbow_joint_position_controller/state", queue_size, get_position_elbow);
    subscribers[3] = n.subscribe("/wrist_1_joint_position_controller/state", queue_size, get_position_wrist_1);
    subscribers[4] = n.subscribe("/wrist_2_joint_position_controller/state", queue_size, get_position_wrist_2);
    subscribers[5] = n.subscribe("/wrist_3_joint_position_controller/state", queue_size, get_position_wrist_3);
}

static void set_publishers(ros::NodeHandle n)
{
    publishers[0] = n.advertise<std_msgs::Float64>("/shoulder_pan_joint_position_controller/command", queue_size);
	publishers[1] = n.advertise<std_msgs::Float64>("/shoulder_lift_joint_position_controller/command", queue_size);
	publishers[2] = n.advertise<std_msgs::Float64>("/elbow_joint_position_controller/command", queue_size);
	publishers[3] = n.advertise<std_msgs::Float64>("/wrist_1_joint_position_controller/command", queue_size);
	publishers[4] = n.advertise<std_msgs::Float64>("/wrist_2_joint_position_controller/command", queue_size);
	publishers[5] = n.advertise<std_msgs::Float64>("/wrist_3_joint_position_controller/command", queue_size);
}

/**
 * @brief Set the new joint values according to the matrix given in input
 * 
 * @param joint_values 
 */
void set_joint_values(vector<long double> positions)
{
    vector<std_msgs::Float64> theta(JOINT_NUM);
    
    cout << MAGENTA << "[set_joint_values] Publishing values..." << endl;

    for(int i = 0; i < JOINT_NUM; i++)
    {
        while (publishers[i].getNumSubscribers() < 1) {
            ros::WallDuration sleep_t(0.5);    
            sleep_t.sleep();
        }
        // After connection are guaranteed to be established, we can publish messages now.
        theta[i].data = positions[i];
        cout << "[set_joint_values] " << positions[i] << endl;
        publishers[i].publish(theta[i]);
        ros::spinOnce();
        
    }    
}


int main (int argc, char **argv)
{
    pair<Vector3ld, Matrix3ld> forward;
    Matrix86ld inverse;
    Matrix6ld jacobian;
    Vector3ld position;

    cout << "[main] Starting ROS node...\n";

    ros::init(argc, argv, "node");
    ros::NodeHandle nodeHandle;

    ros::Rate loop_rate(LOOP_RATE_FREQUENCY);

    set_subscribers(nodeHandle);
    set_publishers(nodeHandle);     

    loop_rate.sleep();

    //just the first time, to set the initial position of the robot
    if(ros::ok())
    {
        jointState = {0, -1.5, 1, 0, 0, 0};
        set_joint_values(jointState);
    
        ros::spinOnce();
        loop_rate.sleep();
    }

    while(ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();
        loop_rate.sleep();
        
        print_joints(jointState);
        
        forward = computeForwardKinematics(jointState);
        print_position(forward.first(0), forward.first(1), forward.first(2));
        Vector3ld ea = matrix2euler(forward.second);
        cout << BLUE << "[main] Roll Pitch Yaw - eluler angles: " << NC << endl << ea << endl;

        inverse = computeInverseKinematics(forward.first, forward.second);
        cout << BLUE << "[main] Inverse Kinematic Matrix: " << NC << endl << inverse << endl;

        // jacobian = computeJacobian(jointState);
        // cout << BLUE << "Jacobian matrix: " << NC << endl << jacobian << endl;

        cout << endl << MAGENTA << "----------------------------------------------------------" << NC << endl << endl;

        /*test for the robot movement*/        
        cout << "[main] Robot movement, please insert x y z values..." << endl;
        cout << "X: ";
        cin >> position(0);
        cout << "Y: ";
        cin >> position(1);
        cout << "Z: ";
        cin >> position(2);
        cout << "[main] Moving robot in position..." << endl << GREEN << position << NC << endl;

        inverse = computeInverseKinematics(position, computeForwardKinematics(jointState).second);
        
        Vector6ld theta = inverse.row(0);
        cout << BLUE << "[main] Desired joint_values: " << MAGENTA << endl << theta << NC << endl;
        vector<long double> tmp {theta.data(), theta.data() + theta.size()}; //gets the first row of the inverse matrix
        set_joint_values(tmp); 

        sleep(1);
        loop_rate.sleep();
    }

    ROS_ERROR("[main] Ros not working\n");
    return -1; 
}