/**
 * @file kinematics.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-04-10
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "utils.h"

pair<Vector3ld, Matrix3ld> computeForwardKinematics(vector<long double>);
Matrix86ld computeInverseKinematics(Vector3ld, Matrix3ld);
Matrix6ld computeJacobian(vector<long double>);
tuple<MatrixXld, MatrixXld, MatrixXld> p2pMotionPlan_(Vector6ld, pair<Vector3ld, Matrix3ld>, long double, long double, long double); 

#endif //KINEMATICS_H