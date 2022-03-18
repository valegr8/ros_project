#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <stack>
#include <algorithm>
#include <set>
#include <limits>
#include <unordered_set>
#include <cmath>
#include <string>
#include <chrono>
#include <thread>

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>

#include <std_msgs/Float64.h>
#include <control_msgs/JointControllerState.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <signal.h>
#include <stdlib.h>
#include <stdio.h>

using namespace Eigen;
using namespace std;

//---------------------------------------------

typedef Matrix<long double, 4, 4> Matrix4ld;
typedef Matrix<long double, 3, 3> Matrix3ld;
typedef Matrix<long double, 8, 6> Matrix86ld;
typedef Matrix<long double, 3, 1> Vector3ld;
typedef Matrix<long double, 4, 1> Vector4ld;
typedef Matrix<long double, Dynamic, Dynamic> MatrixXld;

//---------------------------------------------

//COSTANTI

//Stringe colori
const string GREEN = "\033[0;92m";
const string RED = "\033[0;91m";
const string BLUE = "\033[0;94m";
const string YELLOW = "\033[0;33m";
const string MAGENTA = "\033[0;95m";
const string NC = "\033[0m"; // No Color

const int Njoint = 6;

const int LOOP_RATE_VAL = 10;

const double PI = 3.14159265359;
const double PIMEZZI = 1.57079632679;

const vector<double> Ai = {0.0, -0.425, -0.3922, 0.0, 0.0, 0.0};
const vector<double> ALPHAi = {0.0, PI/2, 0.0, 0.0, PI/2, -PI/2};
const vector<double> Di = {0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996};

const bool debug = false;

//---------------------------------------------

//METODI

void get_position_shoulder_pan(const control_msgs::JointControllerState::ConstPtr&);
void get_position_shoulder_lift(const control_msgs::JointControllerState::ConstPtr&);
void get_position_elbow(const control_msgs::JointControllerState::ConstPtr&);
void get_position_wrist_1(const control_msgs::JointControllerState::ConstPtr&);
void get_position_wrist_2(const control_msgs::JointControllerState::ConstPtr&);
void get_position_wrist_3(const control_msgs::JointControllerState::ConstPtr&);

void set_positions(Matrix86ld);

Matrix4ld transform10(long double);
Matrix4ld transform21(long double);
Matrix4ld transform32(long double);
Matrix4ld transform43(long double);
Matrix4ld transform54(long double);
Matrix4ld transform65(long double);

pair<Matrix4ld, Matrix4ld> computeForwardKinematics(int = 5);
pair<Matrix4ld, Matrix4ld> computeForwardKinematicsTwo(int = 5);
Matrix4ld computeForwardKinematicsThree();
Matrix86ld computeInverseKinematics(Vector3ld, Matrix3ld);

void print_position(int);
void print_positions();
void print_reduct_final_position();

void ctrl_c_handler(int);

// --------------------------------------------

//VARIABILI

//Vettore per ricevere messaggi
vector<control_msgs::JointControllerState::ConstPtr> jointState(Njoint);
//Vettore sottoscrizioni
vector <ros::Subscriber> subscribers(Njoint);
//Vettore pubblicatori
vector <ros::Publisher> publishers(Njoint);
//Booleano che indica quando il programma deve terminare
bool toEnd = false;

// --------------------------------------------

int main (int argc, char **argv)
{
    //Cattura ctrl + c
    //signal(SIGINT, ctrl_c_handler);

    cout << MAGENTA << " *** Comandi Robot *** " << NC << endl << endl;
    
    cout << "Start ROS ... " << flush;

    //Inizializzazione
    ros::init(argc, argv, "code_node");

    //Creo il gestore dei nodi e faccio partire ros
    ros::NodeHandle nodeHandle;

    //Setto il loop rate
    ros::Rate loop_rate(LOOP_RATE_VAL);

    //Controllo se ros sta funzionando
    if(!ros::ok)
    {
        cout << RED << "FALIED" << NC << endl;
        exit(1);
    }
    cout << GREEN << "OK" << NC << endl;

    cout << "Registro sottoscrizioni e pubblicatori ... " << flush;

	//Creo le sottoscrizioni a tutti i giunti per ricevere dati 
    subscribers[0] = nodeHandle.subscribe("/shoulder_pan_joint_position_controller/state", 1000, get_position_shoulder_pan);
    subscribers[1] = nodeHandle.subscribe("/shoulder_lift_joint_position_controller/state", 1000, get_position_shoulder_lift);
    subscribers[2] = nodeHandle.subscribe("/elbow_joint_position_controller/state", 1000, get_position_elbow);
    subscribers[3] = nodeHandle.subscribe("/wrist_1_joint_position_controller/state", 1000, get_position_wrist_1);
    subscribers[4] = nodeHandle.subscribe("/wrist_2_joint_position_controller/state", 1000, get_position_wrist_2);
    subscribers[5] = nodeHandle.subscribe("/wrist_3_joint_position_controller/state", 1000, get_position_wrist_3);
	//Creo i pubblicatori per inviare comandi ai giunti
	publishers[0] = nodeHandle.advertise<std_msgs::Float64>("/shoulder_pan_joint_position_controller/command", 1000);
	publishers[1] = nodeHandle.advertise<std_msgs::Float64>("/shoulder_lift_joint_position_controller/command", 1000);
	publishers[2] = nodeHandle.advertise<std_msgs::Float64>("/elbow_joint_position_controller/command", 1000);
	publishers[3] = nodeHandle.advertise<std_msgs::Float64>("/wrist_1_joint_position_controller/command", 1000);
	publishers[4] = nodeHandle.advertise<std_msgs::Float64>("/wrist_2_joint_position_controller/command", 1000);
	publishers[5] = nodeHandle.advertise<std_msgs::Float64>("/wrist_3_joint_position_controller/command", 1000);

    //Controllo se ros sta funzionando
    if(!ros::ok)
    {
        cout << RED << "FALIED" << NC << endl;
        exit(1);
    }
    cout << GREEN << "OK" << NC << endl;

    cout << "Attendo che il robot sia pronto ... " << flush;
   	for(int i=0; i< 2; i++) {
		ros::spinOnce();
	 	loop_rate.sleep();
	}
    if(!ros::ok)
    {
        cout << RED << "FALIED" << NC << endl;
        exit(1);
    }
    cout << GREEN << "OK" << NC << endl;

    //------------------------------------------------------------------------------------------------------------
    string response;

    //Ciclo in stampo la poszione dell'end effector calcolato con la dk
    while(-1)
    {
        if(debug)
            //Stampo la posizione dei giunti 
            //Solo per utenti esperti
            print_positions();

        //Stampo la posizione dell'end effector attuale
        print_reduct_final_position();

        cout << "Di nuovo? (y/n) : ";
        cin >> response;

        if(response != "y")
            break;
        
        //Controllo che tutto stia funzionando correttamente
        cout << "Attendo che il robot sia pronto ... " << flush;
        for(int i=0; i< 2; i++) {
            ros::spinOnce();
            loop_rate.sleep();
        }
        if(!ros::ok)
        {
            cout << RED << "FALIED" << NC << endl;
            exit(1);
        }
        cout << GREEN << "OK" << NC << endl;
    }

    /*
    //Vettore e Matrice con le coordinate del nuovo punto
    Matrix3ld rotation;
    Vector3ld point;
    //Nuove coordinate
    long double x_to, y_to, z_to;

    //Copio la rotazione dell'end effector attuale e la salvo in rotation
    {
        Matrix4ld cur = computeForwardKinematics().second;

        Matrix3ld rot { 
        {cur(0,0), cur(0,1), cur(0,2)},
        {cur(1,0), cur(1,1), cur(1,2)},
        {cur(2,0), cur(2,1), cur(2,2)}
        };

        rotation = rot;
    } 
    
    //Stampo la posizione dell'end effector attuale
    print_reduct_final_position();

    bool stop = false;
    string response;

    //Ciclo in cui il robot si muove
    while(!stop)
    {
        //Chiedo nuove coordinate
        cout << "X: ";
        cin >> x_to;
        cout << "Y: ";
        cin >> y_to;
        cout << "Z: ";
        cin >> z_to;

        //Copio i valori nel vettore point
        point(0) = x_to;
        point(1) = y_to;
        point(2) = z_to;

        cout << "Muovo il robot al punto " << point(0) << " " << point(1) << " " << point(2) << " ... " << flush;
        //Calcolo gli angoli e imposto la posizione
        set_positions(computeInverseKinematics(point, rotation));
        if(!ros::ok)
        {
            cout << RED << "FALIED" << NC << endl;
            exit(1);
        }
        cout << GREEN << "OK" << NC << endl;

        sleep(3);

        //Controllo che tutto stia funzionando correttamente
        cout << "Attendo che il robot sia pronto ... " << flush;
        for(int i=0; i< 2; i++) {
            ros::spinOnce();
            loop_rate.sleep();
        }
        if(!ros::ok)
        {
            cout << RED << "FALIED" << NC << endl;
            exit(1);
        }
        cout << GREEN << "OK" << NC << endl;

        //Stampo la posizione dell'end effector attuale
        print_reduct_final_position();

        cout << "Continui? (y/n) : ";
        cin >> response;

        if(response != "y")
            stop = true;
    }
    */

    cout << MAGENTA << " *** Fine Comandi Robot *** " << NC << endl;

    return 0;
}

void set_positions(Matrix86ld positions)
{
    vector<std_msgs::Float64> theta(Njoint);

    for(int i = 0; i < Njoint; i++)
    {
        theta[i].data = positions(0, i);
        publishers[i].publish(theta[i]);
        ros::spinOnce();
    }    

    return;
}

Matrix86ld computeInverseKinematics(Vector3ld p60, Matrix3ld r60)
{
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
    theta_5_2 = - theta_5_1;
    theta_5_3 = acos( (p60(0)*sin(theta_1_2) - p60(1)*cos(theta_1_2) - Di[3] ) / Di[5]);
    theta_5_4 = - theta_5_3;

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

    t41m = transform10(theta_1_1).inverse() * t60 * transform65(theta_6_1).inverse() * transform54(theta_5_1);
    pair<long double, long double> p41_1 = make_pair(t41m(0, 3), t41m(2, 3)); 
    long double p41xz_1 = hypot(p41_1.first, p41_1.second);

    t41m = transform10(theta_1_1).inverse() * t60 * transform65(theta_6_2).inverse() * transform54(theta_5_2);
    pair<long double, long double> p41_2 = make_pair(t41m(0, 3), t41m(2, 3)); 
    long double p41xz_2 = hypot(p41_2.first, p41_2.second);

    t41m = transform10(theta_1_2).inverse() * t60 * transform65(theta_6_3).inverse() * transform54(theta_5_3);
    pair<long double, long double> p41_3 = make_pair(t41m(0, 3), t41m(2, 3)); 
    long double p41xz_3 = hypot(p41_3.first, p41_3.second);

    t41m = transform10(theta_1_2).inverse() * t60 * transform65(theta_6_4).inverse() * transform54(theta_5_4);
    pair<long double, long double> p41_4 = make_pair(t41m(0, 3), t41m(2, 3)); 
    long double p41xz_4 = hypot(p41_4.first, p41_4.second);

    //Trovo theta 3
    long double theta_3_1, theta_3_2, theta_3_3, theta_3_4, theta_3_5, theta_3_6, theta_3_7, theta_3_8;
    theta_3_1 = acos( (pow(p41xz_1, 2) - pow(Ai[2], 2) - pow(Ai[3], 2) ) / (2 * Ai[2] * Ai[3]) );
    theta_3_2 = acos( (pow(p41xz_2, 2) - pow(Ai[2], 2) - pow(Ai[3], 2) ) / (2 * Ai[2] * Ai[3]) );
    theta_3_3 = acos( (pow(p41xz_3, 2) - pow(Ai[2], 2) - pow(Ai[3], 2) ) / (2 * Ai[2] * Ai[3]) );
    theta_3_4 = acos( (pow(p41xz_4, 2) - pow(Ai[2], 2) - pow(Ai[3], 2) ) / (2 * Ai[2] * Ai[3]) );
    theta_3_5 = -theta_3_1;
    theta_3_6 = -theta_3_2;
    theta_3_7 = -theta_3_3;
    theta_3_8 = -theta_3_4;

    //Trovo theta 2
    long double theta_2_1, theta_2_2, theta_2_3, theta_2_4, theta_2_5, theta_2_6, theta_2_7, theta_2_8;
    theta_2_1 = atan2(-p41_1.second, -p41_1.first) - asin((-Ai[3] * sin(theta_3_1) ) / p41xz_1);
    theta_2_2 = atan2(-p41_2.second, -p41_2.first) - asin((-Ai[3] * sin(theta_3_2) ) / p41xz_2);
    theta_2_3 = atan2(-p41_3.second, -p41_3.first) - asin((-Ai[3] * sin(theta_3_3) ) / p41xz_3);
    theta_2_4 = atan2(-p41_4.second, -p41_4.first) - asin((-Ai[3] * sin(theta_3_4) ) / p41xz_4);
    theta_2_5 = atan2(-p41_1.second, -p41_1.first) - asin(( Ai[3] * sin(theta_3_1) ) / p41xz_1);
    theta_2_6 = atan2(-p41_2.second, -p41_2.first) - asin(( Ai[3] * sin(theta_3_2) ) / p41xz_2);
    theta_2_7 = atan2(-p41_3.second, -p41_3.first) - asin(( Ai[3] * sin(theta_3_3) ) / p41xz_3);
    theta_2_8 = atan2(-p41_4.second, -p41_4.first) - asin(( Ai[3] * sin(theta_3_4) ) / p41xz_4);

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

    Matrix86ld ris 
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

    return ris;
}

pair<Matrix4ld, Matrix4ld> computeForwardKinematics(int joint)
{
    //Parto dalla matrice di identità per calcolare la catena cinametica
    //Calcolo sia la posizione desiderata che quella effettiva

    //Posizione desiderata
    Matrix4ld base_desired 
    {
        {1.0, 0.0, 0.0, 0.0},
        {0.0, 1.0, 0.0, 0.0},
        {0.0, 0.0, 1.0, 0.0},
        {0.0, 0.0, 0.0, 1.0}
    };
    
    //Posizione effettiva
    Matrix4ld base_current 
    {
        {1.0, 0.0, 0.0, 0.0},
        {0.0, 1.0, 0.0, 0.0},
        {0.0, 0.0, 1.0, 0.0},
        {0.0, 0.0, 0.0, 1.0}
    };

    for(int actualJoint = 0; actualJoint <= joint; ++actualJoint)
    {
        //Per ogni giunto moltiplico i valori

        //Posizione desiderata
        Matrix4ld desired 
        { 
            {cos(jointState[actualJoint]->set_point), -sin(jointState[actualJoint]->set_point), 0, Ai[actualJoint]},
            {sin(jointState[actualJoint]->set_point)*cos(ALPHAi[actualJoint]), cos(jointState[actualJoint]->set_point)*cos(ALPHAi[actualJoint]), -sin(ALPHAi[actualJoint]), -Di[actualJoint]*sin(ALPHAi[actualJoint])},
            {sin(jointState[actualJoint]->set_point)*sin(ALPHAi[actualJoint]), cos(jointState[actualJoint]->set_point)*sin(ALPHAi[actualJoint]), cos(ALPHAi[actualJoint]), Di[actualJoint]*cos(ALPHAi[actualJoint])},
            {0, 0, 0, 1}
        };

        //Posizione effettiva
        Matrix4ld current 
        { 
            {cos(jointState[actualJoint]->process_value), -sin(jointState[actualJoint]->process_value), 0, Ai[actualJoint]},
            {sin(jointState[actualJoint]->process_value)*cos(ALPHAi[actualJoint]), cos(jointState[actualJoint]->process_value)*cos(ALPHAi[actualJoint]), -sin(ALPHAi[actualJoint]), -Di[actualJoint]*sin(ALPHAi[actualJoint])},
            {sin(jointState[actualJoint]->process_value)*sin(ALPHAi[actualJoint]), cos(jointState[actualJoint]->process_value)*sin(ALPHAi[actualJoint]), cos(ALPHAi[actualJoint]), Di[actualJoint]*cos(ALPHAi[actualJoint])},
            {0, 0, 0, 1}
        };

        base_desired *= desired;
        base_current *= current;
    }
    
    return make_pair(base_desired, base_current);
}

pair<Matrix4ld, Matrix4ld> computeForwardKinematicsTwo(int joint)
{
    //Posizione desiderata
    Matrix4ld base_desired 
    {
        {1.0, 0.0, 0.0, 0.0},
        {0.0, 1.0, 0.0, 0.0},
        {0.0, 0.0, 1.0, 0.0},
        {0.0, 0.0, 0.0, 1.0}
    };
    
    //Posizione effettiva
    Matrix4ld base_current 
    {
        {1.0, 0.0, 0.0, 0.0},
        {0.0, 1.0, 0.0, 0.0},
        {0.0, 0.0, 1.0, 0.0},
        {0.0, 0.0, 0.0, 1.0}
    };

    for(int i = 0; i < joint; ++i)
    {
        switch (i)
        {
            case 0:
                base_desired *= transform10(jointState[0]->set_point);
                base_current *= transform10(jointState[0]->process_value);
                break;
            case 1:
                base_desired *= transform21(jointState[1]->set_point);
                base_current *= transform21(jointState[1]->process_value);
                break;
            case 2:
                base_desired *= transform32(jointState[2]->set_point);
                base_current *= transform32(jointState[2]->process_value);
                break;
            case 3:
                base_desired *= transform43(jointState[3]->set_point);
                base_current *= transform43(jointState[3]->process_value);
                break;
            case 4:
                base_desired *= transform54(jointState[4]->set_point);
                base_current *= transform54(jointState[4]->process_value);
                break;
            case 5:
                base_desired *= transform65(jointState[5]->set_point);
                base_current *= transform65(jointState[5]->process_value);
                break;
        }
    }

    //base_desired *= transform10(jointState[0]->set_point) * transform21(jointState[1]->set_point) * transform32(jointState[2]->set_point) * transform43(jointState[3]->set_point) * transform54(jointState[4]->set_point) * transform65(jointState[5]->set_point);
    //base_current *= transform10(jointState[0]->process_value) * transform21(jointState[1]->process_value) * transform32(jointState[2]->process_value) * transform43(jointState[3]->process_value) * transform54(jointState[4]->process_value) * transform65(jointState[5]->process_value);

    return make_pair(base_desired, base_current);
}

Matrix4ld computeForwardKinematicsThree()
{
    //Posizione effettiva
    Matrix4ld base_current 
    {
        {1.0, 0.0, 0.0, 0.0},
        {0.0, 1.0, 0.0, 0.0},
        {0.0, 0.0, 1.0, 0.0},
        {0.0, 0.0, 0.0, 1.0}
    };

    base_current *= transform10(jointState[0]->process_value);
    base_current *= transform21(jointState[1]->process_value);
    base_current *= transform32(jointState[2]->process_value);
    base_current *= transform43(jointState[3]->process_value);
    base_current *= transform54(jointState[4]->process_value);
    base_current *= transform65(jointState[5]->process_value);

    return base_current;
}

void print_reduct_final_position()
{
    Matrix4ld position = computeForwardKinematicsThree();
    cout << BLUE << "Motors: " << NC << " [ ";
    for(auto foo : jointState)
        cout << foo->process_value << " ";
    cout << "]" << endl;
    cout << BLUE << "Position : " << NC << " [" << GREEN << " X" << NC << " -> " << YELLOW << position(0, 3) << NC << "; " << GREEN << "Y" << NC << " -> " << YELLOW << position(1, 3) << NC << "; " << GREEN << "Z" << NC << " -> " << YELLOW << position(2, 3) << NC << "; ] " << endl;
}

void print_position(int joint)
{
    vector <string> joints = { "Shoulder", "Upper Arm", "Forearm", "Wrist_1", "Wrist_2", "Wrist_3"};

    //Calolo la posizione effettiva e quella desiderata sia in un modo che nell'altro
    pair<Matrix4ld, Matrix4ld> tmp = computeForwardKinematics(joint);
    Matrix4ld desired_1 = tmp.first;
    Matrix4ld current_1 = tmp.second;
    tmp = computeForwardKinematicsTwo(joint);
    Matrix4ld desired_2 = tmp.first;
    Matrix4ld current_2 = tmp.second;

    //Stampo le info
    cout << BLUE << joints[joint] << " : " << NC << endl;
    cout << "Position " << RED << "[Metodo 1]" << NC << " (" << GREEN << "desired state" << NC << ") [" << GREEN << " X" << NC << " -> " << YELLOW << desired_1(0, 3) << NC << GREEN << " Y" << NC << " -> " << YELLOW << desired_1(1, 3) << GREEN << " Z" << NC << " -> " << YELLOW << desired_1(2, 3) << NC << " ]; " << endl;
    cout << "Position " << RED << "[Metodo 2]" << NC << " (" << GREEN << "desired state" << NC << ") [" << GREEN << " X" << NC << " -> " << YELLOW << desired_2(0, 3) << NC << GREEN << " Y" << NC << " -> " << YELLOW << desired_2(1, 3) << GREEN << " Z" << NC << " -> " << YELLOW << desired_2(2, 3) << NC << " ]; " << endl;
    cout << "float64 set_point (" << GREEN << "desired state" << NC << ") -> " << YELLOW << jointState[joint]->set_point << NC << "; " << endl;
    cout << "Position " << RED << "[Metodo 1]" << NC << " (" << GREEN << "current state" << NC << ") [" << GREEN << " X" << NC << " -> " << YELLOW << current_1(0, 3) << NC << GREEN << " Y" << NC << " -> " << YELLOW << current_1(1, 3) << GREEN << " Z" << NC << " -> " << YELLOW << current_1(2, 3) << NC << " ]; " << endl;
    cout << "Position " << RED << "[Metodo 2]" << NC << " (" << GREEN << "current state" << NC << ") [" << GREEN << " X" << NC << " -> " << YELLOW << current_2(0, 3) << NC << GREEN << " Y" << NC << " -> " << YELLOW << current_2(1, 3) << GREEN << " Z" << NC << " -> " << YELLOW << current_2(2, 3) << NC << " ]; " << endl;
    cout << "float64 process_value (" << GREEN << "current value" << NC << ") -> " << YELLOW << jointState[joint]->process_value << NC << "; " << endl;
    //cout << "float64 process_value_dot (" << GREEN << "first time-derivative of pv" << NC << ") -> " << YELLOW << jointState[joint]->process_value_dot << NC << "; " << endl;
    cout << "float64 error (" << GREEN << "process_value - set_point" << NC << ") -> " << YELLOW << jointState[joint]->error << NC << "; " << endl;
    //cout << "float64 time_step (" << GREEN << "time between two consecutive updates/execution" << NC << ") -> " << YELLOW << jointState[joint]->time_step << NC << "; " << endl;
    //cout << "float64 command (" << GREEN << "output of the controller" << NC << ") -> " << YELLOW << jointState[joint]->command << NC << "; " << endl;
    //cout << "float64 p (" << GREEN << "PID parameters" << NC << ") -> " << YELLOW << jointState[joint]->p << NC << "; " << endl;
    //cout << "float64 i (" << GREEN << "PID parameters" << NC << ") -> " << YELLOW << jointState[joint]->i << NC << "; " << endl;
    //cout << "float64 d (" << GREEN << "PID parameters" << NC << ") -> " << YELLOW << jointState[joint]->d << NC << "; " << endl;
    //cout << "float64 i_clamp -> " << YELLOW << jointState[joint]->i_clamp << NC << "; " << endl;
    cout << endl;
}

void print_positions()
{
    cout << endl << BLUE << " *** Stato giunti dettagliato ***" << NC << endl << endl;

    for(int i = 0; i < Njoint; ++i)
        print_position(i);
}

Matrix4ld transform10(long double theta_0)
{
    Matrix4ld actual 
    {
        {cos(theta_0), -sin(theta_0), 0.0, 0.0},
        {sin(theta_0), cos(theta_0), 0.0, 0.0},
        {0.0, 0.0, 1.0, Di[0]},
        {0.0, 0.0, 0.0, 1.0}
    };

    return actual;
}

Matrix4ld transform21(long double theta_1)
{
    Matrix4ld actual 
    {
        {cos(theta_1), -sin(theta_1), 0.0, 0.0},
        {0.0, 0.0, -1.0, 0.0},
        {sin(theta_1), cos(theta_1), 0.0, 0.0},
        {0.0, 0.0, 0.0, 1.0}
    };
    
    return actual;
}

Matrix4ld transform32(long double theta_2)
{
    Matrix4ld actual 
    {
        {cos(theta_2), -sin(theta_2), 0.0, Ai[2]},
        {sin(theta_2), cos(theta_2), 0.0, 0.0},
        {0.0, 0.0, 1.0, 0.0},
        {0.0, 0.0, 0.0, 1.0}
    };
    
    return actual;
}

Matrix4ld transform43(long double theta_3)
{
    Matrix4ld actual 
    {
        {cos(theta_3), -sin(theta_3), 0.0, Ai[3]},
        {sin(theta_3), cos(theta_3), 0.0, 0.0},
        {0.0, 0.0, 1.0, Di[3]},
        {0.0, 0.0, 0.0, 1.0}
    };
    
    return actual;
}

Matrix4ld transform54(long double theta_4)
{
    Matrix4ld actual 
    {
        {cos(theta_4), -sin(theta_4), 0.0, 0.0},
        {0.0, 0.0, -1.0, -Di[4]},
        {sin(theta_4), cos(theta_4), 0.0, 0.0},
        {0.0, 0.0, 0.0, 1.0}
    };
    
    return actual;
}

Matrix4ld transform65(long double theta_5)
{
    Matrix4ld actual 
    {
        {cos(theta_5), -sin(theta_5), 0.0, 0.0},
        {0.0, 0.0, 1.0, Di[5]},
        {-sin(theta_5), -cos(theta_5), 0.0, 0.0},
        {0.0, 0.0, 0.0, 1.0}
    };
    
    return actual;
}

void get_position_shoulder_pan(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {jointState[0] = ctr_msg;}
void get_position_shoulder_lift(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {jointState[1] = ctr_msg;}
void get_position_elbow(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {jointState[2] = ctr_msg;}
void get_position_wrist_1(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {jointState[3] = ctr_msg;}
void get_position_wrist_2(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {jointState[4] = ctr_msg;}
void get_position_wrist_3(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {jointState[5] = ctr_msg;}

void ctrl_c_handler(int s){ toEnd = true; }

/*
SPIEGAZIONE
subscribers / publishers [0] -> shoulder_pan (spalla padella)
subscribers / publishers [1] -> shoulder_lift (spalla sollevamento)
subscribers / publishers [2] -> elbow (gomito)
subscribers / publishers [3] -> wrist_1 (polso 1)
subscribers / publishers [4] -> wrist_2 (polso 2)
subscribers / publishers [5] -> wrist_3 (polso 3)
*/