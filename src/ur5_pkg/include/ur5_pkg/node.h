/**
 * @file node.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-03-18
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef NODE_H
#define NODE_H

#include <ur5_pkg/utils.h>


 /********************/
 /* GLOBAL VARIABLES */
 /********************/

const double LOOP_RATE_FREQUENCY = 0.5;  /**< used to set run loops frequency*/

vector<ros::Subscriber> subscribers(JOINT_NUM); /**< global subscribers vector*/
vector<ros::Publisher> publishers(JOINT_NUM);  /**< global publisher vector*/
vector<long double> jointState(JOINT_NUM); /**< contains all /state values of the joints*/
int queue_size; /**< used for publisher and subscribers queue size */

 /********************/
 /* STATIC FUNCTIONS */
 /********************/

static void get_position_shoulder_pan(const control_msgs::JointControllerState::ConstPtr&);
static void get_position_shoulder_lift(const control_msgs::JointControllerState::ConstPtr&);
static void get_position_elbow(const control_msgs::JointControllerState::ConstPtr&);
static void get_position_wrist_1(const control_msgs::JointControllerState::ConstPtr&);
static void get_position_wrist_2(const control_msgs::JointControllerState::ConstPtr&);
static void get_position_wrist_3(const control_msgs::JointControllerState::ConstPtr&);

static void set_publishers(ros::NodeHandle);
static void set_subscribers(ros::NodeHandle);

void print_robot_status();
void set_joint_values(vector<long double>);
bool moveTo(Vector6ld, pair<Vector3ld, Matrix3ld>);

#endif  //NODE_H