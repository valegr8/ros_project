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

#include "ros/ros.h"
#include <control_msgs/JointControllerState.h>
#include <utils.hpp>
#include <ur5_pkg/ForwardKinematic.h>

#define DEBUG 1
// #undef DEBUG

Matrix4ld transform10(long double theta_0)
{
    Matrix4ld actual 
    {
        {cos(theta_0), -sin(theta_0), 0.0, 0.0},
        {sin(theta_0), cos(theta_0), 0.0, 0.0},
        {0.0, 0.0, 1.0, Di[0]},
        {0.0, 0.0, 0.0, 1.0}
    };

    return actual;
}

Matrix4ld transform21(long double theta_1)
{
    Matrix4ld actual 
    {
        {cos(theta_1), -sin(theta_1), 0.0, 0.0},
        {0.0, 0.0, -1.0, Di[1]},
        {sin(theta_1), cos(theta_1), 0.0, 0.0},
        {0.0, 0.0, 0.0, 1.0}
    };
    
    return actual;
}

Matrix4ld transform32(long double theta_2)
{
    Matrix4ld actual 
    {
        {cos(theta_2), -sin(theta_2), 0.0, Ai[1]},
        {sin(theta_2), cos(theta_2), 0.0, 0.0},
        {0.0, 0.0, 1.0, Di[2]},
        {0.0, 0.0, 0.0, 1.0}
    };
    
    return actual;
}

Matrix4ld transform43(long double theta_3)
{
    Matrix4ld actual 
    {
        {cos(theta_3), -sin(theta_3), 0.0, Ai[2]},
        {sin(theta_3), cos(theta_3), 0.0, 0.0},
        {0.0, 0.0, 1.0, Di[3]},
        {0.0, 0.0, 0.0, 1.0}
    };
    
    return actual;
}

Matrix4ld transform54(long double theta_4)
{
    Matrix4ld actual 
    {
        {cos(theta_4), -sin(theta_4), 0.0, 0.0},
        {0.0, 0.0, -1.0, -Di[4]},
        {sin(theta_4), cos(theta_4), 0.0, 0.0},
        {0.0, 0.0, 0.0, 1.0}
    };
    
    return actual;
}

Matrix4ld transform65(long double theta_5)
{
    Matrix4ld actual 
    {
        {cos(theta_5), -sin(theta_5), 0.0, 0.0},
        {0.0, 0.0, 1.0, Di[5]},
        {-sin(theta_5), -cos(theta_5), 0.0, 0.0},
        {0.0, 0.0, 0.0, 1.0}
    };
    
    return actual;
}

void print_matrix(Matrix4ld m)
{
    printf("------------------------------------------------\n");
    printf("%Lf, %Lf, %Lf, %Lf ;\n", m(0,0), m(0,1), m(0,2), m(0,3));
    printf("%Lf, %Lf, %Lf, %Lf ;\n", m(1,0), m(1,1), m(1,2), m(1,3));
    printf("%Lf, %Lf, %Lf, %Lf ;\n", m(2,0), m(2,1), m(2,2), m(2,3));
    printf("%Lf, %Lf, %Lf, %Lf ;\n", m(3,0), m(3,1), m(3,2), m(3,3));
    printf("------------------------------------------------\n");
}

bool forward_kinematic(ur5_pkg::ForwardKinematic::Request  &req, ur5_pkg::ForwardKinematic::Response &res)
{

    Matrix4ld T 
    {
        {1.0, 0.0, 0.0, 0.0},
        {0.0, 1.0, 0.0, 0.0},
        {0.0, 0.0, 1.0, 0.0},
        {0.0, 0.0, 0.0, 1.0}
    };
    

    T = transform10(req.theta0);
#ifdef DEBUG
    printf("\n\nT10:\n");
    print_matrix(T);
#endif
    T *= transform21(req.theta1);
#ifdef DEBUG
    printf("T20:\n");
    print_matrix(T);
#endif
    T *= transform32(req.theta2);
#ifdef DEBUG
    printf("T30:\n");
    print_matrix(T);
#endif
    T *= transform43(req.theta3);
#ifdef DEBUG
    printf("T40:\n");
    print_matrix(T);
#endif
    T *= transform54(req.theta4);
#ifdef DEBUG
    printf("T50:\n");
    print_matrix(T);
#endif
    T *= transform65(req.theta5);

#ifdef DEBUG
    printf("T60:\n");
    print_matrix(T);
#endif

    res.x = T(0,3);
    res.y = T(1,3);
    res.z = T(2,3);

    return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "forward_kinematic");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("ForwardKinematc", forward_kinematic);
#ifdef DEBUG
  ROS_INFO("Forward kinematic\n");
#endif
  ros::spin();
  return 0;
}