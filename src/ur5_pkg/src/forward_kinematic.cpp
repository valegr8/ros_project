/**
 * @file forward_kinematic.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-03-18
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <ur5_pkg/transformation_matrix.h>

pair<Vector3ld, Matrix3ld> computeForwardKinematics(vector<long double> joint)
{
    Matrix4ld matrix 
    {
        {1.0, 0.0, 0.0, 0.0},
        {0.0, 1.0, 0.0, 0.0},
        {0.0, 0.0, 1.0, 0.0},
        {0.0, 0.0, 0.0, 1.0}
    };

    matrix *= transform10(joint[0]);
    matrix *= transform21(joint[1]);
    matrix *= transform32(joint[2]);
    matrix *= transform43(joint[3]);
    matrix *= transform54(joint[4]);
    matrix *= transform65(joint[5]);

    return make_pair(
        Vector3ld {
            matrix(0,3), matrix(1,3), matrix(2,3)
        },     
        Matrix3ld { 
            {matrix(0,0), matrix(0,1), matrix(0,2)},
            {matrix(1,0), matrix(1,1), matrix(1,2)},
            {matrix(2,0), matrix(2,1), matrix(2,2)}
        });
}