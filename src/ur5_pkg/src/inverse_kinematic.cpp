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
#include <ur5_pkg/transformation_matrix.h>

Matrix86ld computeInverseKinematics(Vector3ld p60, Matrix3ld r60)
{
    Matrix4ld t60 
    { 
        {r60(0, 0), r60(0, 1), r60(0, 2), p60(0, 0)},
        {r60(1, 0), r60(1, 1), r60(1, 2), p60(1, 0)},
        {r60(2, 0), r60(2, 1), r60(2, 2), p60(2, 0)},
        {0.0, 0.0, 0.0, 1.0}
    };
    
    //theta 1
    long double theta_1_1, theta_1_2;
    {
        MatrixXld p50 = t60 * (MatrixXld(4,1) << 0.0, 0.0, -Di[5], 1.0 ).finished(); //p05 is a 4 x 1 matrix
        long double phi_1 = atan2(p50(1), p50(0)); //atan2 is arctan(y/x), between [-PI, +PI] 
        long double phi_2 = acos(Di[3]/hypot(p50(1), p50(0))); //hypot is sqrt(x^2 + y^2)

        theta_1_1 = phi_1 + phi_2 + PIMEZZI;
        theta_1_2 = phi_1 - phi_2 + PIMEZZI;
    }

    //theta 5
    long double theta_5_1, theta_5_2, theta_5_3, theta_5_4;
    theta_5_1 = acos( (p60(0)*sin(theta_1_1) - p60(1)*cos(theta_1_1) - Di[3] ) / Di[5]);
    theta_5_2 = -theta_5_1;
    theta_5_3 = acos( (p60(0)*sin(theta_1_2) - p60(1)*cos(theta_1_2) - Di[3] ) / Di[5]);
    theta_5_4 = -theta_5_3;

    //theta 6
    long double theta_6_1, theta_6_2, theta_6_3, theta_6_4;
    {
        Matrix4ld t06 = t60.inverse(); 
        Vector3ld xHat {t06(0, 0), t06(1, 0), t06(2, 0)};
        Vector3ld yHat {t06(0, 1), t06(1, 1), t06(2, 1)};
        theta_6_1 = atan2( ( - xHat(1) * sin(theta_1_1) + yHat(1) * cos(theta_1_1) ) / sin(theta_5_1), (  xHat(0) * sin(theta_1_1) - yHat(0) * cos(theta_1_1) ) / sin(theta_5_1));
        theta_6_2 = atan2( ( - xHat(1) * sin(theta_1_1) + yHat(1) * cos(theta_1_1) ) / sin(theta_5_2), (  xHat(0) * sin(theta_1_1) - yHat(0) * cos(theta_1_1) ) / sin(theta_5_2));
        theta_6_3 = atan2( ( - xHat(1) * sin(theta_1_2) + yHat(1) * cos(theta_1_2) ) / sin(theta_5_3), (  xHat(0) * sin(theta_1_2) - yHat(0) * cos(theta_1_2) ) / sin(theta_5_3));
        theta_6_4 = atan2( ( - xHat(1) * sin(theta_1_2) + yHat(1) * cos(theta_1_2) ) / sin(theta_5_4), (  xHat(0) * sin(theta_1_2) - yHat(0) * cos(theta_1_2) ) / sin(theta_5_4));
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
    theta_3_1 = acos( (pow(p41xz_1, 2) - pow(Ai[1], 2) - pow(Ai[2], 2) ) / (2 * Ai[1] * Ai[2]) );
    theta_3_2 = acos( (pow(p41xz_2, 2) - pow(Ai[1], 2) - pow(Ai[2], 2) ) / (2 * Ai[1] * Ai[2]) );
    theta_3_3 = acos( (pow(p41xz_3, 2) - pow(Ai[1], 2) - pow(Ai[2], 2) ) / (2 * Ai[1] * Ai[2]) );
    theta_3_4 = acos( (pow(p41xz_4, 2) - pow(Ai[1], 2) - pow(Ai[2], 2) ) / (2 * Ai[1] * Ai[2]) );
    theta_3_5 = -theta_3_1;
    theta_3_6 = -theta_3_2;
    theta_3_7 = -theta_3_3;
    theta_3_8 = -theta_3_4;

    //theta 2
    long double theta_2_1, theta_2_2, theta_2_3, theta_2_4, theta_2_5, theta_2_6, theta_2_7, theta_2_8;
    theta_2_1 = atan2(-p41_1.second, -p41_1.first) - asin((-Ai[2] * sin(theta_3_1) ) / p41xz_1);
    theta_2_2 = atan2(-p41_2.second, -p41_2.first) - asin((-Ai[2] * sin(theta_3_2) ) / p41xz_2);
    theta_2_3 = atan2(-p41_3.second, -p41_3.first) - asin((-Ai[2] * sin(theta_3_3) ) / p41xz_3);
    theta_2_4 = atan2(-p41_4.second, -p41_4.first) - asin((-Ai[2] * sin(theta_3_4) ) / p41xz_4);
    theta_2_5 = atan2(-p41_1.second, -p41_1.first) - asin(( Ai[2] * sin(theta_3_1) ) / p41xz_1);
    theta_2_6 = atan2(-p41_2.second, -p41_2.first) - asin(( Ai[2] * sin(theta_3_2) ) / p41xz_2);
    theta_2_7 = atan2(-p41_3.second, -p41_3.first) - asin(( Ai[2] * sin(theta_3_3) ) / p41xz_3);
    theta_2_8 = atan2(-p41_4.second, -p41_4.first) - asin(( Ai[2] * sin(theta_3_4) ) / p41xz_4);

    //theta 4
    Matrix4ld t43m;
    pair<long double, long double> xhat43;
    long double theta_4_1, theta_4_2, theta_4_3, theta_4_4, theta_4_5, theta_4_6, theta_4_7, theta_4_8;

    t43m = transform32(theta_3_1).inverse() * transform21(theta_2_1).inverse() * transform10(theta_1_1).inverse() * t60 * transform65(theta_6_1).inverse() * transform54(theta_5_1).inverse();
    xhat43 = make_pair(t43m(0, 0), t43m(1, 0));
    theta_4_1 = atan2(xhat43.second, xhat43.first);

    t43m = transform32(theta_3_2).inverse() * transform21(theta_2_2).inverse() * transform10(theta_1_1).inverse() * t60 * transform65(theta_6_2).inverse() * transform54(theta_5_2).inverse();
    xhat43 = make_pair(t43m(0, 0), t43m(1, 0));
    theta_4_2 = atan2(xhat43.second, xhat43.first);

    t43m = transform32(theta_3_3).inverse() * transform21(theta_2_3).inverse() * transform10(theta_1_2).inverse() * t60 * transform65(theta_6_3).inverse() * transform54(theta_5_3).inverse();
    xhat43 = make_pair(t43m(0, 0), t43m(1, 0));
    theta_4_3 = atan2(xhat43.second, xhat43.first);

    t43m = transform32(theta_3_4).inverse() * transform21(theta_2_4).inverse() * transform10(theta_1_2).inverse() * t60 * transform65(theta_6_4).inverse() * transform54(theta_5_4).inverse();
    xhat43 = make_pair(t43m(0, 0), t43m(1, 0));
    theta_4_4 = atan2(xhat43.second, xhat43.first);

    t43m = transform32(theta_3_5).inverse() * transform21(theta_2_5).inverse() * transform10(theta_1_1).inverse() * t60 * transform65(theta_6_1).inverse() * transform54(theta_5_1).inverse();
    xhat43 = make_pair(t43m(0, 0), t43m(1, 0));
    theta_4_5 = atan2(xhat43.second, xhat43.first);

    t43m = transform32(theta_3_6).inverse() * transform21(theta_2_6).inverse() * transform10(theta_1_1).inverse() * t60 * transform65(theta_6_2).inverse() * transform54(theta_5_2).inverse();
    xhat43 = make_pair(t43m(0, 0), t43m(1, 0));
    theta_4_6 = atan2(xhat43.second, xhat43.first);

    t43m = transform32(theta_3_7).inverse() * transform21(theta_2_7).inverse() * transform10(theta_1_2).inverse() * t60 * transform65(theta_6_3).inverse() * transform54(theta_5_3).inverse();
    xhat43 = make_pair(t43m(0, 0), t43m(1, 0));
    theta_4_7 = atan2(xhat43.second, xhat43.first);

    t43m = transform32(theta_3_8).inverse() * transform21(theta_2_8).inverse() * transform10(theta_1_2).inverse() * t60 * transform65(theta_6_4).inverse() * transform54(theta_5_4).inverse();
    xhat43 = make_pair(t43m(0, 0), t43m(1, 0));
    theta_4_8 = atan2(xhat43.second, xhat43.first);

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