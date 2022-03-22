/**
 * @file inverse_kinematic.cpp
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
#include <ur5_pkg/InverseKinematic.h>

bool inverse_kinematic_callback(ur5_pkg::InverseKinematic::Request &, ur5_pkg::InverseKinematic::Response &);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "inverse_kinematic");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("InverseKinematic", inverse_kinematic_callback);
  ROS_INFO("Inverse kinematic\n");
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
bool inverse_kinematic_callback(ur5_pkg::InverseKinematic::Request  &req, ur5_pkg::InverseKinematic::Response &res)
{
    ROS_INFO("%f",req.x);
    res.theta0 = 0;
    //TODO: inverse kinematic
    return true;
}
