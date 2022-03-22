/**
 * @file jacobian_kinematic.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-03-21
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include <utils.hpp>
#include <transformation_matrix.hpp>
#include <ur5_pkg/JacobianKinematic.h>

bool jacobian_kinematic_callback(ur5_pkg::JacobianKinematic::Request &, ur5_pkg::JacobianKinematic::Response &);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "jacobian_kinematic");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("JacobianKinematic", jacobian_kinematic_callback);
  ROS_INFO("Jacobian kinematic\n");
  ros::spin();
  return 0;
}

/**
 * @brief 
 * 
 * @param req 
 * @param res 
 * @return true 
 * @return false 
 */
bool jacobian_kinematic_callback(ur5_pkg::JacobianKinematic::Request  &req, ur5_pkg::JacobianKinematic::Response &res)
{
    ROS_INFO("%f",req.input);
    res.output = 0;
    //TODO: Jacobian kinematic
    return true;
}