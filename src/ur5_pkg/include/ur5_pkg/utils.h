#ifndef UTIlS_H
#define UTIlS_H

#include <iostream> //input/output stream
#include <fstream> //file stream
#include <string> //string types
#include <utility> //pair, make_pair, swap
#include <vector> //vector
#include <cmath> //math functions
#include <complex> //complex numbers
#include <limits> //numeric_limits
#include <algorithm> //min, max, sort, search
#include <csignal> //signal
#include <chrono> //millisecond
#include <thread> //thread
#include <tuple> //Tuple

#include <ros/ros.h> //ros header
#include <control_msgs/JointControllerState.h> //Control messages
#include <control_msgs/GripperCommandActionGoal.h> //Gripper control
#include <std_msgs/Float64.h> //Joints and gripper comunication
#include <gazebo_ros_link_attacher/Attach.h> //Dynamic-link plugin header
#include <gazebo_ros_link_attacher/AttachRequest.h> //Dynamic-link plugin header
#include <gazebo_ros_link_attacher/AttachResponse.h> //Dynamic-link plugin header
#include "std_msgs/String.h"
#include <eigen3/Eigen/Core>//Eigen3 header - for matrix calculation
#include <eigen3/Eigen/Dense>

using namespace Eigen;
using namespace std;

//Definizioni di tipi
typedef Matrix<long double, 4, 4> Matrix4ld; // 4x4 matrix
typedef Matrix<long double, 6, 6> Matrix6ld; // 6x6 matrix
typedef Matrix<long double, 6, 4> Matrix64ld; // 6x4 matrix
typedef Matrix<long double, 3, 3> Matrix3ld; // 3x3 matrix
typedef Matrix<long double, 8, 6> Matrix86ld; // 8x6 matrix
typedef Matrix<long double, 3, 1> Vector3ld;  // row vector 3x1
typedef Matrix<long double, 4, 1> Vector4ld; // row  vector 4x1
typedef Matrix<long double, 6, 1> Vector6ld; // row  vector 6x1
typedef Matrix<long double, 7, 1> Vector7ld; // row  vector 7x1
typedef Matrix<long double, Dynamic, Dynamic> MatrixXld; // dynamic matrix

//----------------------------------------------------------------------------------------

//Definizioni costanti
const int JOINT_NUM = 6; //number of joint for ur5
const double LOOP_RATE_FREQUENCY = 30;  //< used to set run loops frequency
const long double PI = 3.14159265359; //pi value
const long double PIMEZZI = 1.57079632679; //pi/2 value
const string GREEN = "\033[0;92m";
const string RED = "\033[0;91m";
const string  BLUE = "\033[0;94m";
const string  YELLOW = "\033[0;33m";
const string  MAGENTA = "\033[0;95m";
const string  NC = "\033[0m"; // No Color
const vector<double> Ai = {0.0, -0.425, -0.39225, 0.0, 0.0, 0.0};   //< Vector that contains a values for ur5, used for kinematic
const vector<double> ALPHAi = {PI/2, 0.0, 0.0, PI/2, -PI/2, 0.0};   //< Vector that contains alpha values for ur5, used for kinematic
const vector<double> Di = {0.089159, 0.0, 0.0, 0.10915, 0.09465, 0.0823};   //< Vector that contains d values for ur5, used for kinematic

//----------------------------------------------------------------------------------------

//Funzioni per i segnali
void set_signals();
void signal_handler(int);

//Funzioni di stampa
void print_position(long double, long double, long double);
void print_eluler_angles(Vector3ld);
void print_joints(vector<long double>);
void print_desidered_joints(Vector6ld);
void print_robot_status();
void print_gripper_status();

//Funzioni di conversione radianti - gradi
Vector3ld radToDeg(Vector3ld);
Vector3ld degToRad(Vector3ld);

//Funzioni di conversione matrici di rotazione - angoli euleriani
Matrix3ld eulerToMatrix(Vector3ld);
Vector3ld matrixToEuler(Matrix3ld);

//Funzioni di get 
void get_position_shoulder_pan(const control_msgs::JointControllerState::ConstPtr&);
void get_position_shoulder_lift(const control_msgs::JointControllerState::ConstPtr&);
void get_position_elbow(const control_msgs::JointControllerState::ConstPtr&);
void get_position_wrist_1(const control_msgs::JointControllerState::ConstPtr&);
void get_position_wrist_2(const control_msgs::JointControllerState::ConstPtr&);
void get_position_wrist_3(const control_msgs::JointControllerState::ConstPtr&);
void get_position_gripper(const control_msgs::JointControllerState::ConstPtr& ctr_msg);

//Funzioni di inizializzazione
void set_joint_values(vector<long double>);
void set_publishers(ros::NodeHandle);
void set_subscribers(ros::NodeHandle);

//Controllo gripper
void gripper_set(long double, ros::Rate&);

//Creazione link dinamici
bool createDynamicLink(ros::NodeHandle, string, string, string = "link", string = "link");
bool destroyDynamicLink(ros::NodeHandle, string, string, string = "link", string = "link");
bool dynamicLinkAction(ros::ServiceClient, string, string, string = "link", string = "link");

//Funzione movimento
bool pointToPointMotionPlan(pair<Vector3ld, Vector3ld>, ros::Rate&, long double, long double, long double);
bool pointToPointMotionPlan(pair<Vector3ld, Matrix3ld>, ros::Rate&, long double, long double, long double);

//Cinematiche
pair<Vector3ld, Matrix3ld> computeForwardKinematics(vector<long double>);
Matrix86ld computeInverseKinematics(Vector3ld, Matrix3ld);
Matrix6ld computeJacobian(vector<long double>);

//Trasformazioni
Matrix4ld transform10(long double);
Matrix4ld transform21(long double);
Matrix4ld transform32(long double);
Matrix4ld transform43(long double);
Matrix4ld transform54(long double);
Matrix4ld transform65(long double);

//Metodi temporanei di testing
void pigliaCuboCentrale(ros::Rate&, ros::NodeHandle);
void askUserGoToPoint(ros::Rate&);
void mySleep(ros::Rate&);

//----------------------------------------------------------------------------------------

extern vector<ros::Subscriber> subscribers; //< global subscribers vector
extern vector<ros::Publisher> publishers;  // global publisher vector
extern ros::Subscriber gripperSubscriber; // gripper subscriber
extern ros::Publisher gripperPublisher; // gripper publisher
extern vector<long double> jointState; // contains all /state values of the joints
extern long double gripperState; // contains state values of the gripper
extern int queue_size; // used for publisher and subscribers queue size 
extern bool debug;
extern bool gripper;

#endif  //UTILS_H
