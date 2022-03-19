/**
 * @file utils.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-03-18
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef UTIlS_HPP
#define UTIlS_HPP

#include <vector>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

using namespace Eigen;
using namespace std;
using namespace ros;


#define GREEN = "\033[0;92m";
#define RED = "\033[0;91m";
#define BLUE = "\033[0;94m";
#define YELLOW = "\033[0;33m";
#define MAGENTA = "\033[0;95m";
#define NC "\033[0m"; // No Color

#define JOINT_NUM 6  /**< number of joint for ur5*/

#define PI 3.14159265359    /**< pi avlue*/
#define PIMEZZI 1.57079632679   /**< pi/2 value*/

static vector<double> Ai = {0.0, -0.425, -0.39225, 0.0, 0.0, 0.0};   /**< Vector that contains a values for ur5, used for kinematic*/
static vector<double> ALPHAi = {PI/2, 0.0, 0.0, PI/2, -PI/2, 0.0};   /**< Vector that contains alpha values for ur5, used for kinematic*/
static vector<double> Di = {0.089159, 0.0, 0.0, 0.10915, 0.09465, 0.0823};   /**< Vector that contains d values for ur5, used for kinematic*/

typedef Matrix<long double, 4, 4> Matrix4ld;

// typedef Matrix<long double, 3, 3> Matrix3ld;
// typedef Matrix<long double, 8, 6> Matrix86ld;
// typedef Matrix<long double, 3, 1> Vector3ld;
// typedef Matrix<long double, 4, 1> Vector4ld;
// typedef Matrix<long double, Dynamic, Dynamic> MatrixXld;

#endif  //UTILS_HPP