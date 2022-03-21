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
#include <iostream>
#include <ros/ros.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

using namespace Eigen;
using namespace std;


#define GREEN "\033[0;92m"
#define RED "\033[0;91m"
#define BLUE "\033[0;94m"
#define YELLOW "\033[0;33m"
#define MAGENTA "\033[0;95m"
#define NC "\033[0m" // No Color

#define JOINT_NUM 6  /**< number of joint for ur5*/

#define PI 3.14159265359    /**< pi avlue*/
#define PIMEZZI 1.57079632679   /**< pi/2 value*/

static vector<double> Ai = {0.0, -0.425, -0.39225, 0.0, 0.0, 0.0};   /**< Vector that contains a values for ur5, used for kinematic*/
static vector<double> ALPHAi = {PI/2, 0.0, 0.0, PI/2, -PI/2, 0.0};   /**< Vector that contains alpha values for ur5, used for kinematic*/
static vector<double> Di = {0.089159, 0.0, 0.0, 0.10915, 0.09465, 0.0823};   /**< Vector that contains d values for ur5, used for kinematic*/

typedef Matrix<long double, 4, 4> Matrix4ld;    /**< 4x4 matrix*/
typedef Matrix<long double, 3, 3> Matrix3ld;    /**< 3x3 matrix*/
typedef Matrix<long double, 8, 6> Matrix86ld;   /**< 8x6 matrix*/
typedef Matrix<long double, 3, 1> Vector3ld;    /**< row vector 3x1*/
typedef Matrix<long double, 4, 1> Vector4ld;    /**< row  vector 4x1*/
typedef Matrix<long double, Dynamic, Dynamic> MatrixXld;    /**< dynamic matrix*/

/**
 * @brief utility function that prints a 4x4 matrix
 * 
 * @param m 4x4 matrix
 */
void print_matrix(Matrix4ld m)
{
    printf("------------------------------------------------\n");
    printf("%Lf, %Lf, %Lf, %Lf ;\n", m(0,0), m(0,1), m(0,2), m(0,3));
    printf("%Lf, %Lf, %Lf, %Lf ;\n", m(1,0), m(1,1), m(1,2), m(1,3));
    printf("%Lf, %Lf, %Lf, %Lf ;\n", m(2,0), m(2,1), m(2,2), m(2,3));
    printf("%Lf, %Lf, %Lf, %Lf ;\n", m(3,0), m(3,1), m(3,2), m(3,3));
    printf("------------------------------------------------\n");
}

/**
 * @brief utility function that prints the given position
 * 
 * @param x 
 * @param y 
 * @param z 
 */
void print_position(double x, double y, double z)
{
    cout << BLUE << "Position: " << NC << " [ ";
    cout << YELLOW << x << NC << ", "; 
    cout << YELLOW << y << NC << ", ";
    cout << YELLOW << z << NC << " ] ";
    cout << endl;
}

/**
 * @brief utility function that prints the given values of the joints
 * 
 * @param joints vector
 */
void print_joints(vector<double> joints)
{
    cout << BLUE << "Joints: " << NC << " [ ";
    for(auto it : joints)
        cout << it << " ";
    cout << "]" << endl;
}

#endif  //UTILS_HPP