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
#include <transformation_matrix.hpp>
#include <ur5_pkg/ForwardKinematic.h>

bool forward_kinematic_callback(ur5_pkg::ForwardKinematic::Request &, ur5_pkg::ForwardKinematic::Response &);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "forward_kinematic");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("ForwardKinematic", forward_kinematic_callback);
  ROS_INFO("Forward kinematic\n");
  ros::spin();

  return 0;
}

/**
 * @brief service callback function that computes the forward kinematic for ur5 robot
 * 
 * @param req the values of the six joints of ur5
 * @param res the pose of the end effector (double x, double y, double z)
 * @return true if succesful
 * @return false 
 */
bool forward_kinematic_callback(ur5_pkg::ForwardKinematic::Request &req, ur5_pkg::ForwardKinematic::Response &res)
{
    Matrix4ld matrix
    {
        {1.0, 0.0, 0.0, 0.0},
        {0.0, 1.0, 0.0, 0.0},
        {0.0, 0.0, 1.0, 0.0},
        {0.0, 0.0, 0.0, 1.0}
    };

    matrix = transform10(req.theta0);
    if(debug)
      cout << "Transformation10:" << endl << matrix << endl;

    matrix *= transform21(req.theta1);
    if(debug)
      cout << "Transformation21:" << endl << matrix << endl;

    matrix *= transform32(req.theta2);
    if(debug)
      cout << "Transformation32:" << endl << matrix << endl;    

    matrix*= transform43(req.theta3);
    if(debug)
      cout << "Transformation43:" << endl << matrix << endl;

    matrix*= transform54(req.theta4);
    if(debug)
      cout << "Transformation54:" << endl << matrix << endl;    

    matrix*= transform65(req.theta5);
    if(debug)
      cout << "Transformation65:" << endl << matrix << endl;    

    res.x = matrix(0,3);
    res.y = matrix(1,3);
    res.z = matrix(2,3);

    return true;
}

