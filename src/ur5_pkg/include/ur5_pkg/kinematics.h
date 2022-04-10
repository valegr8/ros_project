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
    
#endif //KINEMATICS_H