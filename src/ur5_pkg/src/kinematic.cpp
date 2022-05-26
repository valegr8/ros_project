#include "ur5_pkg/utils.h"

bool pointToPointMotionPlan(pair<Vector3ld, Vector3ld> end, ros::Rate& loop_rate, long double minT, long double maxT, long double deltaT)
{
    return pointToPointMotionPlan(make_pair(end.first, eulerToMatrix(end.second)), loop_rate, minT, maxT, deltaT);
}

bool pointToPointMotionPlan(pair<Vector3ld, Matrix3ld> end, ros::Rate& loop_rate, long double minT, long double maxT, long double deltaT)
{
    vector<std_msgs::Float64> theta(JOINT_NUM);//Mandatory to public
    Matrix64ld A;//For next joint position

    {
        //Gets joints actual value
        ros::spinOnce();
        loop_rate.sleep();
        Vector6ld ik_start(jointState.data());

        //Inverting y
        end.first(0) = -end.first(0);
        end.first(1) = -end.first(1);
        end.first(2) = end.first(2);

        //Gets joints final value
        Vector6ld ik_end = computeInverseKinematics(end.first, end.second).row(0);

        Matrix4ld m 
        {
            {1.0, minT, pow(minT, 2.0), pow(minT, 3.0)},
            {0.0, 1.0, 2*minT, 3*pow(minT, 2.0)},
            {1.0, maxT, pow(maxT, 2.0), pow(maxT, 3.0)},
            {0.0, 1.0, 2*maxT, 3*pow(maxT, 2.0)}
        };
        Matrix4ld mInverse = m.inverse();

        for(int i = 0; i < JOINT_NUM; i++)
        {       
            Vector4ld b { ik_start(i), 0.0, ik_end(i), 0.0};
            A.row(i) = mInverse * b;
        }
    }

    for(long double t = minT; t < maxT + deltaT; t+=deltaT)
    {           
        //cout << t << " ";
        //Compute the next position for all joints
        for(int j = 0; j < JOINT_NUM; j++)
        {
            //theta[j].data = fmod(A(j,0) + A(j,1)*t + A(j,2)*pow(t, 2) + A(j,3)*pow(t, 3), PI);
            theta[j].data = A(j,0) + A(j,1)*t + A(j,2)*pow(t, 2) + A(j,3)*pow(t, 3), PI;
            //Checking for NaN
            if(isnan(theta[j].data))
                return false;
            //cout << theta[j].data << " ";
        }
        //cout << endl;
        
        //Public
        for(int j = 0; j < JOINT_NUM; j++)
            publishers[j].publish(theta[j]);

        ros::spinOnce();
        loop_rate.sleep();
    }
   
   return true;
}

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
            -matrix(0,3), -matrix(1,3), matrix(2,3)
        },     
        Matrix3ld { 
            {matrix(0,0), matrix(0,1), matrix(0,2)},
            {matrix(1,0), matrix(1,1), matrix(1,2)},
            {matrix(2,0), matrix(2,1), matrix(2,2)}
        });
}

Matrix86ld computeInverseKinematics(Vector3ld p60, Matrix3ld r60)
{
    Matrix4ld t60 
    { 
        {r60(0, 0), r60(0, 1), r60(0, 2), p60(0, 0)},
        {r60(1, 0), r60(1, 1), r60(1, 2), p60(1, 0)},
        {r60(2, 0), r60(2, 1), r60(2, 2), p60(2, 0)},
        {0.0, 0.0, 0.0, 1.0}
    };

    complex<long double> complexConverter(1.0,0.0);
    
    //theta 1
    long double theta_1_1, theta_1_2;
    {
        MatrixXld p50 = t60 * (MatrixXld(4,1) << 0.0, 0.0, -Di[5], 1.0 ).finished(); //p05 is a 4 x 1 matrix
        complex<long double> phi_1 = atan2(p50(1), p50(0))* complexConverter; //atan2 is arctan(y/x), between [-PI, +PI] 
        complex<long double> phi_2 = acos(Di[3]/hypot(p50(1), p50(0))) * complexConverter; //hypot is sqrt(x^2 + y^2)

        theta_1_1 = real(phi_1 + phi_2 + PIMEZZI);
        theta_1_2 = real(phi_1 - phi_2 + PIMEZZI);
    }

    //theta 5
    long double theta_5_1, theta_5_2, theta_5_3, theta_5_4;
    theta_5_1 = real(acos( (p60(0)*sin(theta_1_1) - p60(1)*cos(theta_1_1) - Di[3] ) / Di[5] * complexConverter));
    theta_5_2 = -theta_5_1;
    theta_5_3 = real(acos( (p60(0)*sin(theta_1_2) - p60(1)*cos(theta_1_2) - Di[3] ) / Di[5] * complexConverter));
    theta_5_4 = -theta_5_3;

    //theta 6
    long double theta_6_1, theta_6_2, theta_6_3, theta_6_4;
    {
        Matrix4ld t06 = t60.inverse(); 
        Vector3ld xHat {t06(0, 0), t06(1, 0), t06(2, 0)};
        Vector3ld yHat {t06(0, 1), t06(1, 1), t06(2, 1)};
        theta_6_1 = real(atan2( ( - xHat(1) * sin(theta_1_1) + yHat(1) * cos(theta_1_1) ) / sin(theta_5_1), (  xHat(0) * sin(theta_1_1) - yHat(0) * cos(theta_1_1) ) / sin(theta_5_1)) * complexConverter);
        theta_6_2 = real(atan2( ( - xHat(1) * sin(theta_1_1) + yHat(1) * cos(theta_1_1) ) / sin(theta_5_2), (  xHat(0) * sin(theta_1_1) - yHat(0) * cos(theta_1_1) ) / sin(theta_5_2)) * complexConverter);
        theta_6_3 = real(atan2( ( - xHat(1) * sin(theta_1_2) + yHat(1) * cos(theta_1_2) ) / sin(theta_5_3), (  xHat(0) * sin(theta_1_2) - yHat(0) * cos(theta_1_2) ) / sin(theta_5_3)) * complexConverter);
        theta_6_4 = real(atan2( ( - xHat(1) * sin(theta_1_2) + yHat(1) * cos(theta_1_2) ) / sin(theta_5_4), (  xHat(0) * sin(theta_1_2) - yHat(0) * cos(theta_1_2) ) / sin(theta_5_4)) * complexConverter);
    }
    
    Matrix4ld t41m;

    t41m = transform10(theta_1_1).inverse() * t60 * transform65(theta_6_1).inverse() * transform54(theta_5_1).inverse();
    pair<long double, long double> p41_1 = make_pair(t41m(0, 3), t41m(2, 3)); 
    long double p41xz_1 = hypot(p41_1.first, p41_1.second);

    t41m = transform10(theta_1_1).inverse() * t60 * transform65(theta_6_2).inverse() * transform54(theta_5_2).inverse();
    pair<long double, long double> p41_2 = make_pair(t41m(0, 3), t41m(2, 3)); 
    long double p41xz_2 = hypot(p41_2.first, p41_2.second);

    t41m = transform10(theta_1_2).inverse() * t60 * transform65(theta_6_3).inverse() * transform54(theta_5_3).inverse();
    pair<long double, long double> p41_3 = make_pair(t41m(0, 3), t41m(2, 3)); 
    long double p41xz_3 = hypot(p41_3.first, p41_3.second);

    t41m = transform10(theta_1_2).inverse() * t60 * transform65(theta_6_4).inverse() * transform54(theta_5_4).inverse();
    pair<long double, long double> p41_4 = make_pair(t41m(0, 3), t41m(2, 3)); 
    long double p41xz_4 = hypot(p41_4.first, p41_4.second);

    //theta 3
    long double theta_3_1, theta_3_2, theta_3_3, theta_3_4, theta_3_5, theta_3_6, theta_3_7, theta_3_8;
    theta_3_1 = real( acos( (pow(p41xz_1, 2) - pow(Ai[1], 2) - pow(Ai[2], 2) ) / (2 * Ai[1] * Ai[2]) * complexConverter));
    theta_3_2 = real( acos( (pow(p41xz_2, 2) - pow(Ai[1], 2) - pow(Ai[2], 2) ) / (2 * Ai[1] * Ai[2]) * complexConverter));
    theta_3_3 = real( acos( (pow(p41xz_3, 2) - pow(Ai[1], 2) - pow(Ai[2], 2) ) / (2 * Ai[1] * Ai[2]) * complexConverter));
    theta_3_4 = real( acos( (pow(p41xz_4, 2) - pow(Ai[1], 2) - pow(Ai[2], 2) ) / (2 * Ai[1] * Ai[2]) * complexConverter));
    theta_3_5 = -theta_3_1;
    theta_3_6 = -theta_3_2;
    theta_3_7 = -theta_3_3;
    theta_3_8 = -theta_3_4;

    //theta 2
    long double theta_2_1, theta_2_2, theta_2_3, theta_2_4, theta_2_5, theta_2_6, theta_2_7, theta_2_8;
    theta_2_1 = real(atan2(-p41_1.second, -p41_1.first) * complexConverter - asin((-Ai[2] * sin(theta_3_1) ) / p41xz_1 * complexConverter) * complexConverter);
    theta_2_2 = real(atan2(-p41_2.second, -p41_2.first) * complexConverter - asin((-Ai[2] * sin(theta_3_2) ) / p41xz_2 * complexConverter) * complexConverter);
    theta_2_3 = real(atan2(-p41_3.second, -p41_3.first) * complexConverter - asin((-Ai[2] * sin(theta_3_3) ) / p41xz_3 * complexConverter) * complexConverter);
    theta_2_4 = real(atan2(-p41_4.second, -p41_4.first) * complexConverter - asin((-Ai[2] * sin(theta_3_4) ) / p41xz_4 * complexConverter) * complexConverter);
    theta_2_5 = real(atan2(-p41_1.second, -p41_1.first) * complexConverter - asin(( Ai[2] * sin(theta_3_1) ) / p41xz_1 * complexConverter) * complexConverter);
    theta_2_6 = real(atan2(-p41_2.second, -p41_2.first) * complexConverter - asin(( Ai[2] * sin(theta_3_2) ) / p41xz_2 * complexConverter) * complexConverter);
    theta_2_7 = real(atan2(-p41_3.second, -p41_3.first) * complexConverter - asin(( Ai[2] * sin(theta_3_3) ) / p41xz_3 * complexConverter) * complexConverter);
    theta_2_8 = real(atan2(-p41_4.second, -p41_4.first) * complexConverter - asin(( Ai[2] * sin(theta_3_4) ) / p41xz_4 * complexConverter) * complexConverter);

    //theta 4
    Matrix4ld t43m;
    pair<long double, long double> xhat43;
    long double theta_4_1, theta_4_2, theta_4_3, theta_4_4, theta_4_5, theta_4_6, theta_4_7, theta_4_8;

    t43m = transform32(theta_3_1).inverse() * transform21(theta_2_1).inverse() * transform10(theta_1_1).inverse() * t60 * transform65(theta_6_1).inverse() * transform54(theta_5_1).inverse();
    xhat43 = make_pair(t43m(0, 0), t43m(1, 0));
    theta_4_1 = real(atan2(xhat43.second, xhat43.first) * complexConverter);

    t43m = transform32(theta_3_2).inverse() * transform21(theta_2_2).inverse() * transform10(theta_1_1).inverse() * t60 * transform65(theta_6_2).inverse() * transform54(theta_5_2).inverse();
    xhat43 = make_pair(t43m(0, 0), t43m(1, 0));
    theta_4_2 = real(atan2(xhat43.second, xhat43.first) * complexConverter);

    t43m = transform32(theta_3_3).inverse() * transform21(theta_2_3).inverse() * transform10(theta_1_2).inverse() * t60 * transform65(theta_6_3).inverse() * transform54(theta_5_3).inverse();
    xhat43 = make_pair(t43m(0, 0), t43m(1, 0));
    theta_4_3 = real(atan2(xhat43.second, xhat43.first) * complexConverter);

    t43m = transform32(theta_3_4).inverse() * transform21(theta_2_4).inverse() * transform10(theta_1_2).inverse() * t60 * transform65(theta_6_4).inverse() * transform54(theta_5_4).inverse();
    xhat43 = make_pair(t43m(0, 0), t43m(1, 0));
    theta_4_4 = real(atan2(xhat43.second, xhat43.first) * complexConverter);

    t43m = transform32(theta_3_5).inverse() * transform21(theta_2_5).inverse() * transform10(theta_1_1).inverse() * t60 * transform65(theta_6_1).inverse() * transform54(theta_5_1).inverse();
    xhat43 = make_pair(t43m(0, 0), t43m(1, 0));
    theta_4_5 = real(atan2(xhat43.second, xhat43.first) * complexConverter);

    t43m = transform32(theta_3_6).inverse() * transform21(theta_2_6).inverse() * transform10(theta_1_1).inverse() * t60 * transform65(theta_6_2).inverse() * transform54(theta_5_2).inverse();
    xhat43 = make_pair(t43m(0, 0), t43m(1, 0));
    theta_4_6 = real(atan2(xhat43.second, xhat43.first) * complexConverter);

    t43m = transform32(theta_3_7).inverse() * transform21(theta_2_7).inverse() * transform10(theta_1_2).inverse() * t60 * transform65(theta_6_3).inverse() * transform54(theta_5_3).inverse();
    xhat43 = make_pair(t43m(0, 0), t43m(1, 0));
    theta_4_7 = real(atan2(xhat43.second, xhat43.first) * complexConverter);

    t43m = transform32(theta_3_8).inverse() * transform21(theta_2_8).inverse() * transform10(theta_1_2).inverse() * t60 * transform65(theta_6_4).inverse() * transform54(theta_5_4).inverse();
    xhat43 = make_pair(t43m(0, 0), t43m(1, 0));
    theta_4_8 = real(atan2(xhat43.second, xhat43.first) * complexConverter);

    return Matrix86ld  
    {
        {theta_1_1, theta_2_1, theta_3_1, theta_4_1, theta_5_1, theta_6_1},
        {theta_1_1, theta_2_2, theta_3_2, theta_4_2, theta_5_2, theta_6_2},
        {theta_1_2, theta_2_3, theta_3_3, theta_4_3, theta_5_3, theta_6_3},
        {theta_1_2, theta_2_4, theta_3_4, theta_4_4, theta_5_4, theta_6_4},
        {theta_1_1, theta_2_5, theta_3_5, theta_4_5, theta_5_1, theta_6_1},
        {theta_1_1, theta_2_6, theta_3_6, theta_4_6, theta_5_2, theta_6_2},
        {theta_1_2, theta_2_7, theta_3_7, theta_4_7, theta_5_3, theta_6_3},
        {theta_1_2, theta_2_8, theta_3_8, theta_4_8, theta_5_4, theta_6_4}
    };
}

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