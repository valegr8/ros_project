/**
 * @file diff_kinematic.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-03-21
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include <ur5_pkg/transformation_matrix.h>

Matrix6ld computeJacobian(vector<long double> joint)
{
    long double th1 = joint[0];
    long double th2 = joint[1];
    long double th3 = joint[2];
    long double th4 = joint[3];
    long double th5 = joint[4];
    long double th6 = joint[5];

    vector<long double> j1(6), j2(6), j3(6), j4(6), j5(6), j6(6);

    j1[0] = Di[4]*(cos(th1)*cos(th5) + cos(th2 + th3 + th4)*sin(th1)*sin(th5)) + Di[2]*cos(th1) + Di[3]*cos(th1) - Ai[2]*cos(th2 + th3)*sin(th1) - Ai[1]*cos(th2)*sin(th1) - Di[4]*sin(th2 + th3 + th4)*sin(th1);;
    j1[1] = Di[4]*(cos(th5)*sin(th1) - cos(th2 + th3 + th4)*cos(th1)*sin(th5)) + Di[2]*sin(th1) + Di[3]*sin(th1) + Ai[2]*cos(th2 + th3)*cos(th1) + Ai[1]*cos(th1)*cos(th2) + Di[4]*sin(th2 + th3 + th4)*cos(th1);;
    j1[2] = 0;
    j1[3] = 0;
    j1[4] = 0;
    j1[5] = 1;

    j2[0] = -cos(th1)*(Ai[2]*sin(th2 + th3) + Ai[1]*sin(th2) + Di[4]*(sin(th2 + th3)*sin(th4) - cos(th2 + th3)*cos(th4)) - Di[4]*sin(th5)*(cos(th2 + th3)*sin(th4) + sin(th2 + th3)*cos(th4)));
    j2[1] = -sin(th1)*(Ai[2]*sin(th2 + th3) + Ai[1]*sin(th2) + Di[4]*(sin(th2 + th3)*sin(th4) - cos(th2 + th3)*cos(th4)) - Di[4]*sin(th5)*(cos(th2 + th3)*sin(th4) + sin(th2 + th3)*cos(th4)));
    j2[2] = Ai[2]*cos(th2 + th3) - (Di[4]*sin(th2 + th3 + th4 + th5))/2 + Ai[1]*cos(th2) + (Di[4]*sin(th2 + th3 + th4 - th5))/2 + Di[4]*sin(th2 + th3 + th4);
    j2[3] = sin(th1);
    j2[4] = -cos(th1);
    j2[5] = 0;

    j3[0] = cos(th1)*(Di[4]*cos(th2 + th3 + th4) - Ai[2]*sin(th2 + th3) + Di[4]*sin(th2 + th3 + th4)*sin(th5));
    j3[1] = sin(th1)*(Di[4]*cos(th2 + th3 + th4) - Ai[2]*sin(th2 + th3) + Di[4]*sin(th2 + th3 + th4)*sin(th5));
    j3[2] = Ai[2]*cos(th2 + th3) - (Di[4]*sin(th2 + th3 + th4 + th5))/2 + (Di[4]*sin(th2 + th3 + th4 - th5))/2 + Di[4]*sin(th2 + th3 + th4);
    j3[3] = sin(th1);
    j3[4] = -cos(th1);
    j3[5] = 0;

    j4[0] = Di[4]*cos(th1)*(cos(th2 + th3 + th4) + sin(th2 + th3 + th4)*sin(th5));
    j4[1] = Di[4]*sin(th1)*(cos(th2 + th3 + th4) + sin(th2 + th3 + th4)*sin(th5));
    j4[2] = Di[4]*(sin(th2 + th3 + th4 - th5)/2 + sin(th2 + th3 + th4) - sin(th2 + th3 + th4 + th5)/2);
    j4[3] = sin(th1);
    j4[4] = -cos(th1);
    j4[5] = 0;
     
    j5[0] = -Di[4]*sin(th1)*sin(th5) - Di[4]*cos(th2 + th3 + th4)*cos(th1)*cos(th5);
    j5[1] = Di[4]*cos(th1)*sin(th5) - Di[4]*cos(th2 + th3 + th4)*cos(th5)*sin(th1);
    j5[2] = -Di[4]*(sin(th2 + th3 + th4 - th5)/2 + sin(th2 + th3 + th4 + th5)/2);
    j5[3] = sin(th2 + th3 + th4)*cos(th1);
    j5[4] = sin(th2 + th3 + th4)*sin(th1);
    j5[5] = -cos(th2 + th3 + th4);

    j6[0] = 0;
    j6[1] = 0;
    j6[2] = 0;
    j6[3] = cos(th5)*sin(th1) - cos(th2 + th3 + th4)*cos(th1)*sin(th5);;
    j6[4] = -cos(th1)*cos(th5) - cos(th2 + th3 + th4)*sin(th1)*sin(th5);
    j6[5] = -sin(th2 + th3 + th4)*sin(th5);

    return Matrix6ld
    {
        {j1[0], j2[0], j3[0], j4[0], j5[0], j6[0]},
        {j1[1], j2[1], j3[1], j4[1], j5[1], j6[1]},
        {j1[2], j2[2], j3[2], j4[2], j5[2], j6[2]},
        {j1[3], j2[3], j3[3], j4[3], j5[3], j6[3]},
        {j1[4], j2[4], j3[4], j4[4], j5[4], j6[4]},
        {j1[5], j2[5], j3[5], j4[5], j5[5], j6[5]}
    };
}