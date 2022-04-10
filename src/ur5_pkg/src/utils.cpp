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
    cout << BLUE << "Position " << NC << "( x y z ) : [ ";
    cout << YELLOW << x << NC << ", "; 
    cout << YELLOW << y << NC << ", ";
    cout << YELLOW << z << NC << " ] ";
    cout << endl;
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