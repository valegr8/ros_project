/**
 * @file node.cpp
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
#include <ur5_pkg/InverseKinematic.h>
#include <ur5_pkg/JacobianKinematic.h>

#define LOOP_RATE_FREQUENCY 0.5  /**< used to set run loops frequency*/

vector<ros::Subscriber> subscribers(JOINT_NUM); /**< global subscribers vector*/
vector<long double> jointState(JOINT_NUM); /**< contains all /state values of the joints*/

bool call_jk_service(ros::NodeHandle);
bool call_fk_service(ros::NodeHandle);
bool call_fk_service(ros::NodeHandle);

//Funzioni di test
//NON MODIFICARE GLI INPUT/OUTPUT
pair<Vector3ld, Matrix3ld> computeForwardKinematics(vector<long double>);
Matrix86ld computeInverseKinematics(Vector3ld, Matrix3ld);
Matrix6ld computeJacobian(vector<long double>);

void get_position_shoulder_pan(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {jointState[0] = ctr_msg->process_value;}
void get_position_shoulder_lift(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {jointState[1] = ctr_msg->process_value;}
void get_position_elbow(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {jointState[2] = ctr_msg->process_value;}
void get_position_wrist_1(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {jointState[3] = ctr_msg->process_value;}
void get_position_wrist_2(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {jointState[4] = ctr_msg->process_value;}
void get_position_wrist_3(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {jointState[5] = ctr_msg->process_value;}

void set_subscribers(ros::NodeHandle n)
{
    int queue_size = 1000;
    subscribers[0] = n.subscribe("/shoulder_pan_joint_position_controller/state", queue_size, get_position_shoulder_pan);
    subscribers[1] = n.subscribe("/shoulder_lift_joint_position_controller/state", queue_size, get_position_shoulder_lift);
    subscribers[2] = n.subscribe("/elbow_joint_position_controller/state", queue_size, get_position_elbow);
    subscribers[3] = n.subscribe("/wrist_1_joint_position_controller/state", queue_size, get_position_wrist_1);
    subscribers[4] = n.subscribe("/wrist_2_joint_position_controller/state", queue_size, get_position_wrist_2);
    subscribers[5] = n.subscribe("/wrist_3_joint_position_controller/state", queue_size, get_position_wrist_3);
}

int main (int argc, char **argv)
{
    cout << "Starting ROS\n";

    ros::init(argc, argv, "node");
    ros::NodeHandle nodeHandle;
    ros::Rate loop_rate(LOOP_RATE_FREQUENCY);

    set_subscribers(nodeHandle);
    /*
    while(ros::ok())
    {
        ros::spinOnce();
        
        //call_fk_service(nodeHandle);
        
        //call_ik_service(nodeHandle);
        //call_jk_service(nodeHandle);
        
        loop_rate.sleep();

        sleep(5);
    }
    */

    //---------------------------------------------------------
    //test_function
    while(ros::ok())
    {
        ros::spinOnce();

        //Stampo giunti
        print_joints(jointState);
        
        //Calcolo cinematica diretta
        pair<Vector3ld, Matrix3ld> diretta = computeForwardKinematics(jointState);
        //Stampo la posizione e la matrice di rotazione
        print_position(diretta.first(0), diretta.first(1), diretta.first(2));
        cout << BLUE << "Matrice Rotazione: " << NC << endl << diretta.second << endl;

        //Calcolo cinematica inversa
        Matrix86ld inversa = computeInverseKinematics(diretta.first, diretta.second);
        //Stampo matrice cinematica inversa
        cout << BLUE << "Matrice cinematica inversa: " << NC << endl << inversa << endl;

        //Calcolo Jacobian
        Matrix6ld jacobian = computeJacobian(jointState);
        //Stampo matrice jacobian
        cout << BLUE << "Matrice Jacobian: " << NC << endl << jacobian << endl;

        cout << endl << MAGENTA << "----------------------------------------------------------------" << NC << endl << endl;

        loop_rate.sleep();
        sleep(3);
    }

    ROS_ERROR("Ros not working\n");
    return 1; 
}

//--------------------------------------------------------------------------
//Funzioni di test

pair<Vector3ld, Matrix3ld> computeForwardKinematics(vector<long double> joint)
{
    //Per Valeria
    //in input ci sono i 6 giunti
    //in output ci sono 2 matrici: una 1x3 (matrice posizione) (oppure 3 long double) ed una 3x3 (matrice di rotazione)

    //Posizione effettiva
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
            matrix(0,3), matrix(1,3), matrix(2,3)
        },     
        Matrix3ld { 
            {matrix(0,0), matrix(0,1), matrix(0,2)},
            {matrix(1,0), matrix(1,1), matrix(1,2)},
            {matrix(2,0), matrix(2,1), matrix(2,2)}
        });
}

//Funzionante
Matrix86ld computeInverseKinematics(Vector3ld p60, Matrix3ld r60)
{
    //Per Valeria
    //in input ci sono 2 matrici: una 1x3 (matrice posizione) (oppure 3 long double) ed una 3x3 (matrice di rotazione)
    //in output una matrice 8x6

    //Construisco la matrice t60 4 x 4
    Matrix4ld t60 
    { 
        {r60(0, 0), r60(0, 1), r60(0, 2), p60(0, 0)},
        {r60(1, 0), r60(1, 1), r60(1, 2), p60(1, 0)},
        {r60(2, 0), r60(2, 1), r60(2, 2), p60(2, 0)},
        {0.0, 0.0, 0.0, 1.0}
    };
    
    //Trovo theta 1
    long double theta_1_1, theta_1_2;
    {
        MatrixXld p50 = t60 * (MatrixXld(4,1) << 0.0, 0.0, -Di[5], 1.0 ).finished(); //p05 è una matrice 4 x 1
        long double phi_1 = atan2(p50(1), p50(0)); //atan2 è arctan(y/x) nell'intervallo [-PI, +PI] 
        long double phi_2 = acos(Di[3]/hypot(p50(1), p50(0))); //hypot è sqrt(x^2 + y^2)

        theta_1_1 = phi_1 + phi_2 + PIMEZZI;
        theta_1_2 = phi_1 - phi_2 + PIMEZZI;
    }

    //Trovo theta 5
    long double theta_5_1, theta_5_2, theta_5_3, theta_5_4;
    theta_5_1 = acos( (p60(0)*sin(theta_1_1) - p60(1)*cos(theta_1_1) - Di[3] ) / Di[5]);
    theta_5_2 = -theta_5_1;
    theta_5_3 = acos( (p60(0)*sin(theta_1_2) - p60(1)*cos(theta_1_2) - Di[3] ) / Di[5]);
    theta_5_4 = -theta_5_3;

    //Trovo theta 6
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
    
    //Calcoli per gli angoli successivi
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

    //Trovo theta 3
    long double theta_3_1, theta_3_2, theta_3_3, theta_3_4, theta_3_5, theta_3_6, theta_3_7, theta_3_8;
    theta_3_1 = acos( (pow(p41xz_1, 2) - pow(Ai[1], 2) - pow(Ai[2], 2) ) / (2 * Ai[1] * Ai[2]) );
    theta_3_2 = acos( (pow(p41xz_2, 2) - pow(Ai[1], 2) - pow(Ai[2], 2) ) / (2 * Ai[1] * Ai[2]) );
    theta_3_3 = acos( (pow(p41xz_3, 2) - pow(Ai[1], 2) - pow(Ai[2], 2) ) / (2 * Ai[1] * Ai[2]) );
    theta_3_4 = acos( (pow(p41xz_4, 2) - pow(Ai[1], 2) - pow(Ai[2], 2) ) / (2 * Ai[1] * Ai[2]) );
    theta_3_5 = -theta_3_1;
    theta_3_6 = -theta_3_2;
    theta_3_7 = -theta_3_3;
    theta_3_8 = -theta_3_4;

    //Trovo theta 2
    long double theta_2_1, theta_2_2, theta_2_3, theta_2_4, theta_2_5, theta_2_6, theta_2_7, theta_2_8;
    theta_2_1 = atan2(-p41_1.second, -p41_1.first) - asin((-Ai[2] * sin(theta_3_1) ) / p41xz_1);
    theta_2_2 = atan2(-p41_2.second, -p41_2.first) - asin((-Ai[2] * sin(theta_3_2) ) / p41xz_2);
    theta_2_3 = atan2(-p41_3.second, -p41_3.first) - asin((-Ai[2] * sin(theta_3_3) ) / p41xz_3);
    theta_2_4 = atan2(-p41_4.second, -p41_4.first) - asin((-Ai[2] * sin(theta_3_4) ) / p41xz_4);
    theta_2_5 = atan2(-p41_1.second, -p41_1.first) - asin(( Ai[2] * sin(theta_3_1) ) / p41xz_1);
    theta_2_6 = atan2(-p41_2.second, -p41_2.first) - asin(( Ai[2] * sin(theta_3_2) ) / p41xz_2);
    theta_2_7 = atan2(-p41_3.second, -p41_3.first) - asin(( Ai[2] * sin(theta_3_3) ) / p41xz_3);
    theta_2_8 = atan2(-p41_4.second, -p41_4.first) - asin(( Ai[2] * sin(theta_3_4) ) / p41xz_4);

    //Trovo theta 4

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

Matrix6ld computeJacobian(vector<long double> joint)
{
    //Per Valeria
    //in input ci sono i 6 giunti
    //in output una matrice 6x6

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

//-----------------------------------------------------------------------------------
//Puttanate inutili al momento

/**
 * @brief Create a client object and calls forward kinematic service
 * 
 * @param n NodeHandle
 * @return true if successful, false otherrwise
 */
bool call_fk_service(ros::NodeHandle n)
{
    //<ForwardKinematic> defines the type of the service (.srv), "ForwardKinematc" is the name (defined in .cpp)
    ros::ServiceClient client = n.serviceClient<ur5_pkg::ForwardKinematic>("ForwardKinematic");
    ur5_pkg::ForwardKinematic srv;

    //Passing data to the service
    srv.request.theta0 = jointState[0];
    srv.request.theta1 = jointState[1];
    srv.request.theta2 = jointState[2];
    srv.request.theta3 = jointState[3];
    srv.request.theta4 = jointState[4];
    srv.request.theta5 = jointState[5];

    if(debug)
        print_joints(jointState);

    //Computing forward kinematic
    bool ok = client.call(srv);
    if (ok)
    {
        print_position(srv.response.x,srv.response.y,srv.response.z);
    }
    else
    {
        ROS_ERROR("Failed to call service\n");
        return false;
    }

    return true;
}

bool call_ik_service(ros::NodeHandle n)
{
    ros::ServiceClient client = n.serviceClient<ur5_pkg::InverseKinematic>("InverseKinematic");
    ur5_pkg::InverseKinematic srv;

    //Passing data to the service
    srv.request.x = 1.0;
    srv.request.y = 2.0;
    srv.request.z = 3.0;

    if (client.call(srv))
    {
        cout << RED <<"Inverse kinematic joints: "<<endl;
        print_joints(jointState);
    }
    else
    {
        ROS_ERROR("Failed to call service\n");
        return false;
    }

    return true;
}

bool call_jk_service(ros::NodeHandle n)
{
    ros::ServiceClient client = n.serviceClient<ur5_pkg::JacobianKinematic>("JacobianKinematic");
    ur5_pkg::JacobianKinematic srv;

    //Passing data to the service
    srv.request.input = 1.0;

    if (client.call(srv))
    {
        cout << RED << "Jacobian output: " << srv.response.output << NC <<endl;
    }
    else
    {
        ROS_ERROR("Failed to call service\n");
        return false;
    }

    return true;
}