/**
 * @file trasformation_matrix.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-03-21
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <utils.hpp>

Matrix4ld transform10(long double);
Matrix4ld transform21(long double);
Matrix4ld transform32(long double);
Matrix4ld transform43(long double);
Matrix4ld transform54(long double);
Matrix4ld transform65(long double);


Matrix4ld transform10(long double theta_0)
{
    return Matrix4ld 
    {
        {cos(theta_0), -sin(theta_0), 0.0, 0.0},
        {sin(theta_0), cos(theta_0), 0.0, 0.0},
        {0.0, 0.0, 1.0, Di[0]},
        {0.0, 0.0, 0.0, 1.0}
    };
}

Matrix4ld transform21(long double theta_1)
{
    return Matrix4ld 
    {
        {cos(theta_1), -sin(theta_1), 0.0, 0.0},
        {0.0, 0.0, -1.0, Di[1]},
        {sin(theta_1), cos(theta_1), 0.0, 0.0},
        {0.0, 0.0, 0.0, 1.0}
    };
}

Matrix4ld transform32(long double theta_2)
{
    return Matrix4ld 
    {
        {cos(theta_2), -sin(theta_2), 0.0, Ai[1]},
        {sin(theta_2), cos(theta_2), 0.0, 0.0},
        {0.0, 0.0, 1.0, Di[2]},
        {0.0, 0.0, 0.0, 1.0}
    };
}

Matrix4ld transform43(long double theta_3)
{
    return Matrix4ld 
    {
        {cos(theta_3), -sin(theta_3), 0.0, Ai[2]},
        {sin(theta_3), cos(theta_3), 0.0, 0.0},
        {0.0, 0.0, 1.0, Di[3]},
        {0.0, 0.0, 0.0, 1.0}
    };
}

Matrix4ld transform54(long double theta_4)
{
    return Matrix4ld 
    {
        {cos(theta_4), -sin(theta_4), 0.0, 0.0},
        {0.0, 0.0, -1.0, -Di[4]},
        {sin(theta_4), cos(theta_4), 0.0, 0.0},
        {0.0, 0.0, 0.0, 1.0}
    };
}

Matrix4ld transform65(long double theta_5)
{
    return Matrix4ld 
    {
        {cos(theta_5), -sin(theta_5), 0.0, 0.0},
        {0.0, 0.0, 1.0, Di[5]},
        {-sin(theta_5), -cos(theta_5), 0.0, 0.0},
        {0.0, 0.0, 0.0, 1.0}
    };
}