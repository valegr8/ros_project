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

#include <ros/ros.h>
#include <utils.hpp>

#include <control_msgs/JointControllerState.h>
#include <stdio.h>
#include <ur5_pkg/ForwardKinematic.h>

#define DEBUG 1 /**< used for debug print*/

#define LOOP_RATE_FREQUENCY 10  /**< used to set run loops frequency*/

static vector<ros::Subscriber> subscribers(JOINT_NUM); /**< global subscribers vector*/
static double jointState[JOINT_NUM]; /**< contains all /state values of the joints*/

void get_position_shoulder_pan(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {jointState[0] = ctr_msg->process_value;}
void get_position_shoulder_lift(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {jointState[1] = ctr_msg->process_value;}
void get_position_elbow(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {jointState[2] = ctr_msg->process_value;}
void get_position_wrist_1(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {jointState[3] = ctr_msg->process_value;}
void get_position_wrist_2(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {jointState[4] = ctr_msg->process_value;}
void get_position_wrist_3(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {jointState[5] = ctr_msg->process_value;}


void set_subscribers(ros::NodeHandle n)
{
    int queue_size = 1000;
    subscribers[0] = n.subscribe("/shoulder_pan_joint_position_controller/state", queue_size, get_position_shoulder_pan);
    subscribers[1] = n.subscribe("/shoulder_lift_joint_position_controller/state", queue_size, get_position_shoulder_lift);
    subscribers[2] = n.subscribe("/elbow_joint_position_controller/state", queue_size, get_position_elbow);
    subscribers[3] = n.subscribe("/wrist_1_joint_position_controller/state", queue_size, get_position_wrist_1);
    subscribers[4] = n.subscribe("/wrist_2_joint_position_controller/state", queue_size, get_position_wrist_2);
    subscribers[5] = n.subscribe("/wrist_3_joint_position_controller/state", queue_size, get_position_wrist_3);
}

/**
 * @brief Create a client object and calls forward kinematic service
 * 
 * @param n NodeHandle
 * @return true if successful, false otherrwise
 */
bool call_fk_service(ros::NodeHandle n)
{
    //<ForwardKinematic> defines the type of the service (.srv), "ForwardKinematc" is the name (defined in .cpp)
    ros::ServiceClient client = n.serviceClient<ur5_pkg::ForwardKinematic>("ForwardKinematc");
    ur5_pkg::ForwardKinematic srv;

    //Coping data
    srv.request.theta0 = jointState[0];
    srv.request.theta1 = jointState[1];
    srv.request.theta2 = jointState[2];
    srv.request.theta3 = jointState[3];
    srv.request.theta4 = jointState[4];
    srv.request.theta5 = jointState[5];

    cout << BLUE << "Joint: " << NC << " [ ";
    for(auto foo : jointState)
        cout << foo << " ";
    cout << "]" << endl;

    if (client.call(srv))
    {
        cout << BLUE << "Position : " << NC << " [";
        cout << GREEN << " X" << NC << " -> " << YELLOW << srv.response.x << NC << "; "; 
        cout << GREEN << "Y" << NC << " -> " << YELLOW << srv.response.y << NC << "; ";
        cout << GREEN << "Z" << NC << " -> " << YELLOW << srv.response.z << NC << "; ] ";
        cout << endl;
    }
    else
    {
        ROS_ERROR("Failed to call service\n");
        return false;
    }

    return true;
}

int main (int argc, char **argv)
{
    cout << "Starting ROS\n";

    ros::init(argc, argv, "node");
    ros::NodeHandle nodeHandle;
    ros::Rate loop_rate(LOOP_RATE_FREQUENCY);

    set_subscribers(nodeHandle);

    
    while(ros::ok())
    {
        ros::spinOnce();
        call_fk_service(nodeHandle);
        loop_rate.sleep();

        //Sleeping for 5 sec
        sleep(5);
    }

    ROS_ERROR("Ros not working\n");
    return 1; 
}
