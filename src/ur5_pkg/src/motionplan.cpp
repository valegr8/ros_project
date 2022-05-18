/**
 * @file motionplan.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-03-18
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <ur5_pkg/transformation_matrix.h>
#include <ur5_pkg/kinematics.h>

/**
 * @brief Coputing the movement to a specific position
 * 
 * @param jointsActualState
 * @param endPosition
 * @param minT
 * @param maxT
 * @param deltaT
 */
tuple<MatrixXld, MatrixXld, MatrixXld> p2pMotionPlan_(Vector6ld ik_start, pair<Vector3ld, Matrix3ld> end, long double minT, long double maxT, long double deltaT)
{
    int matrixDimension = (int)round((maxT-minT)/deltaT) + 1;

    MatrixXld Th(matrixDimension, 7);
    MatrixXld xE(matrixDimension, 4);
    MatrixXld phiE(matrixDimension, 4);

    Vector6ld ik_end = computeInverseKinematics(end.first, end.second).row(0);

    Matrix64ld A;
    {
        Matrix4ld m 
        {
            {1.0, minT, pow(minT, 2.0), pow(minT, 3.0)},
            {0.0, 1.0, 2*minT, 3*pow(minT, 2.0)},
            {1.0, maxT, pow(maxT, 2.0), pow(maxT, 3.0)},
            {0.0, 1.0, 2*maxT, 3*pow(maxT, 2.0)}
        };
        Matrix4ld mInverse = m.inverse();

        for(int i = 0; i < 6; i++)
        {       
            Vector4ld b { ik_start(i), 0.0, ik_end(i), 0.0};
            A.row(i) = mInverse * b;
        }
    }

    {
        long double t;
        int i;
        vector<long double> joints(6);
        Vector3ld eulerAngles;
        pair<Vector3ld, Matrix3ld> dk;

        for(t = minT, i = 0; t < maxT + deltaT; t+=deltaT, i++)
        {
            Th(i, 0) = xE(i, 0) = phiE(i, 0) = t;
             
            for(int j = 0; j < 6; j++)
                Th(i, j+1) = joints[j] = A(j,0) + A(j,1)*t + A(j,2)*t*t + A(j,3)*t*t*t;
                
            dk = computeForwardKinematics(joints);
            eulerAngles = dk.second.eulerAngles(2, 1, 0);

            for(int j = 0; j < 3; j++)
            {
                xE(i, j+1) = dk.first(j);
                phiE(i, j+1) = eulerAngles(j);
            }
        }
    }
    
   return make_tuple(Th, xE, phiE);
}

/*
tuple<MatrixXld, MatrixXld, MatrixXld> p2pMotionPlan(pair<Vector3ld, Matrix3ld> start, pair<Vector3ld, Matrix3ld> end, long double minT, long double maxT, long double deltaT)
{
    int matrixDimension = (int)round((maxT-minT)/deltaT) + 1;

    MatrixXld Th(matrixDimension, 7);
    MatrixXld xE(matrixDimension, 4);
    MatrixXld phiE(matrixDimension, 4);

    Vector6ld ik_start= computeInverseKinematics(start.first, start.second).row(0);
    Vector6ld ik_end = computeInverseKinematics(end.first, end.second).row(0);

    //Calcolo matrice A
    Matrix64ld A;
    {
        Matrix4ld m 
        {
            {1.0, minT, pow(minT, 2.0), pow(minT, 3.0)},
            {0.0, 1.0, 2*minT, 3*pow(minT, 2.0)},
            {1.0, maxT, pow(maxT, 2.0), pow(maxT, 3.0)},
            {0.0, 1.0, 2*maxT, 3*pow(maxT, 2.0)}
        };
        Matrix4ld mInverse = m.inverse();

        for(int i = 0; i < 6; i++)
        {       
            Vector4ld b { ik_start(i), 0.0, ik_end(i), 0.0};
            A.row(i) = mInverse * b;
        }
    }

    {
        long double t;
        int i;
        vector<long double> joints(6);
        Vector3ld eulerAngles;
        pair<Vector3ld, Matrix3ld> dk;

        for(t = minT, i = 0; t < maxT + deltaT; t+=deltaT, i++)
        {
            Th(i, 0) = xE(i, 0) = phiE(i, 0) = t;
             
            for(int j = 0; j < 6; j++)
                Th(i, j+1) = joints[j] = A(j,0) + A(j,1)*t + A(j,2)*t*t + A(j,3)*t*t*t;
                
            dk = computeForwardKinematics(joints);
            eulerAngles = dk.second.eulerAngles(2, 1, 0);

            for(int j = 0; j < 3; j++)
            {
                xE(i, j+1) = dk.first(j);
                phiE(i, j+1) = eulerAngles(j);
            }
        }
    }
    
   return make_tuple(Th, xE, phiE);
}

*/
