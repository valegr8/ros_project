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

#include <utils.hpp>
#include <ur5_pkg/ForwardKinematic.h>
#include <transformation_matrix.hpp>


/**
 * @brief service callback function that computes the forward kinematic for ur5 robot
 * 
 * @param req the values of the six joints of ur5
 * @param res the pose of the end effector (double x, double y, double z)
 * @return true if succesful
 * @return false 
 */
bool forward_kinematic_callback(ur5_pkg::ForwardKinematic::Request  &req, ur5_pkg::ForwardKinematic::Response &res)
{

    Matrix4ld T 
    {
        {1.0, 0.0, 0.0, 0.0},
        {0.0, 1.0, 0.0, 0.0},
        {0.0, 0.0, 1.0, 0.0},
        {0.0, 0.0, 0.0, 1.0}
    };
    

    T = transform10(req.theta0);
    printf("\n\nT10:\n");
    print_matrix(T);
    T *= transform21(req.theta1);
    printf("T20:\n");
    print_matrix(T);
    T *= transform32(req.theta2);
    printf("T30:\n");
    print_matrix(T);
    T *= transform43(req.theta3);
    printf("T40:\n");
    print_matrix(T);
    T *= transform54(req.theta4);
    printf("T50:\n");
    print_matrix(T);
    T *= transform65(req.theta5);
    printf("T60:\n");
    print_matrix(T);

    res.x = T(0,3);
    res.y = T(1,3);
    res.z = T(2,3);

    return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "forward_kinematic");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("ForwardKinematic", forward_kinematic_callback);
  ROS_INFO("Forward kinematic\n");
  ros::spin();
  return 0;
}