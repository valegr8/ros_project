/**
 * @file utils.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-03-18
 * 
 * @copyright Copyright (c) 2022
 */

#ifndef UTIlS_H
#define UTIlS_H

#include <iostream> //input/output stream
#include <fstream> //file stream
#include <string> //string types
#include <utility> //pair, make_pair, swap
#include <vector> //vector
#include <cmath> //math functions
#include <limits> //numeric_limits
#include <algorithm> //min, max, sort, search
#include <csignal> //signal
#include <chrono> //millisecond
#include <thread> //thread
#include <tuple> //Tuple

#include <ros/ros.h> //ros header
#include <control_msgs/JointControllerState.h> //Control messages

#include <eigen3/Eigen/Core>//Eigen3 header - for matrix calculation
#include <eigen3/Eigen/Dense>

#include <std_msgs/Float64.h>

using namespace Eigen;
using namespace std;

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

const int JOINT_NUM = 6; //number of joint for ur5

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

void print_position(long double, long double, long double);
void print_eluler_angles(Vector3ld);
void print_joints(vector<long double>);
void print_desidered_joints(Vector6ld);
Matrix3ld euler2matrix(long double roll, long double pitch, long double yaw);
Vector3ld matrix2euler(Matrix3ld m);

#endif  //UTILS_H