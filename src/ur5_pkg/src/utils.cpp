/**
 * @file utils.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-04-10
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <ur5_pkg/utils.h>



/**
 * @brief utility function that prints the given position
 * 
 * @param x 
 * @param y 
 * @param z 
 */
void print_position(long double x, long double y, long double z)
{
    cout << BLUE << "Position ( x y z ) : " << NC << "[ ";
    cout << YELLOW << x << NC << ", "; 
    cout << YELLOW << y << NC << ", ";
    cout << YELLOW << z << NC << " ] ";
    cout << endl;
}

/**
 * @brief utility function that prints the given euler angles
 * 
 * @param euler_angles
 */
void print_eluler_angles(Vector3ld ea)
{
    cout << BLUE << "Eluler angles - Roll(z) Pitch(y) Yaw(x): " << NC << "[ ";
    cout << ea(0) << ", ";
    cout << ea(1) << ", ";
    cout << ea(2) << " ]" << endl;
}

/**
 * @brief utility function that prints the given values of the joints
 * 
 * @param joints vector
 */
void print_joints(vector<long double> joints)
{
    cout << BLUE << "Joints: " << NC << " [ ";
    for(auto it : joints)
        cout << it << " ";
    cout << "]" << endl;
}

/**
 * @brief utility function that prints the given desidered values of the joints
 * 
 * @param joints vector
 */
void print_desidered_joints(Vector6ld joints)
{
    cout << BLUE << "Desired joint values: " << NC << " [ ";
    for(auto it : joints)
        cout << it << " ";
    cout << "]" << endl;
}

/**
 * @brief convertion from euler angles to rotation matrix
 * 
 */
Matrix3ld euler2matrix(long double roll, long double pitch, long double yaw)
{
    AngleAxis<long double> rollAngle(roll, Vector3ld::UnitZ());
    AngleAxis<long double> yawAngle(yaw, Vector3ld::UnitY());
    AngleAxis<long double> pitchAngle(pitch, Vector3ld::UnitX());

    Eigen::Quaternion<long double> q = rollAngle * yawAngle * pitchAngle;

    Matrix3ld rotationMatrix = q.matrix();

    return rotationMatrix;
}

/**
 * @brief convertion from rotation matrix to euler angle
 * 
 * @param m rotation matrix
 * @return Vector3ld euler angles, roll pith yaw
 */
Vector3ld matrix2euler(Matrix3ld m) 
{
    return m.eulerAngles(0, 1, 2);
}

