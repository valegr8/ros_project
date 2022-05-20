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

#include <ur5_pkg/node.h>
#include <ur5_pkg/utils.h>
#include <ur5_pkg/transformation_matrix.h>
#include <ur5_pkg/kinematics.h>
#include <std_msgs/Float64.h>

bool debug = true;

int main (int argc, char **argv)
{
    cout << BLUE << "Starting ROS node..." << NC;

    ros::init(argc, argv, "node");
    ros::NodeHandle nodeHandle;

    ros::Rate loop_rate(LOOP_RATE_FREQUENCY);

    set_subscribers(nodeHandle);
    set_publishers(nodeHandle);     

    loop_rate.sleep();

    cout << GREEN << " [ DONE ] " << endl;

    //just the first time, to set the initial position of the robot
    if(ros::ok())
    {
        jointState = {0, -1.5, 1, 0, 0, 0};
        set_joint_values(jointState);
    
        ros::spinOnce();
        loop_rate.sleep();
    }

    cout << endl << "----------------------------------------------------------" << endl << endl;


    //Now we start
    pair<Vector3ld, Matrix3ld> to; //Per la posizione finale

    while(ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();
        loop_rate.sleep();

        //Printing informations about the robot
        print_robot_status();

        //test for the robot movement
        cout << "To move the robot please insert x y z values..." << endl;
        cout << "X: ";
        cin >> to.first(0);
        cout << "Y: ";
        cin >> to.first(1);
        cout << "Z: ";
        cin >> to.first(2);
        cout << BLUE << "Moving robot to position: " << NC << "[ " << to.first(0) << ", " << to.first(1) << ", " << to.first(2) << " ]" << endl;

        //Temporaneamente prendo la rotazione dell'end effector attuale
        to.second = computeForwardKinematics(jointState).second;
         
        //Copio la posizione dei joint
        //Vector6ld thetaJoint(jointState.data());

        /*
        if(moveTo(Vector6ld (jointState.data()), to))
            cout << GREEN << "[ DONE ]" << NC << endl;
        else
            cout << RED << "[ FAILED ]" << NC << endl;
        */

        cout << endl << "----------------------------------------------------------" << endl << endl;

        sleep(1);
        loop_rate.sleep();
    }

    ROS_ERROR("Ros not working\n");
    return 1; 
    
}

/*
bool moveTo(Vector6ld start, pair<Vector3ld, Matrix3ld> end)
{
    //Computing the movement
    MatrixXld th = get<0>(p2pMotionPlan_(start, end, 0.0, 1.0, 0.01));

    //Mandatory to public
    vector<std_msgs::Float64> theta(6);

    for(int i = 0; i < th.rows(); i++)
    {
        //cout << th(i, 0) << " ";
        for(int j = 0; j < 6; j++)
        {
            //Check for errors
            if(isnan(th(i, j+1)))
                return false;

            //Copy data
            theta[j].data = fmod(th(i, j+1), PI);

            //cout << theta[j].data << " ";

            //Wait for connections
            while (publishers[j].getNumSubscribers() < 1) 
            {
                ros::WallDuration sleep_t(0.5);    
                sleep_t.sleep();
            }

            //Now we can publish messages
            publishers[j].publish(theta[j]);

            //Just to be on the safe side
            ros::spinOnce();
            ros::spinOnce();
            ros::spinOnce();
            ros::spinOnce();
            ros::spinOnce();
            ros::spinOnce();
        }
        //cout << endl;
    }

    return true;
}
*/

/**
 * @brief Set the new joint values according to the matrix given in input
 * 
 * @param joint_values 
 */
void set_joint_values(vector<long double> positions)
{
    vector<std_msgs::Float64> theta(JOINT_NUM);
    
    cout << BLUE << "Setting joints values " << NC;

    cout << " [ ";
    for(int i = 0; i < JOINT_NUM; i++)
    {
        while (publishers[i].getNumSubscribers() < 1) 
        {
            ros::WallDuration sleep_t(0.5);    
            sleep_t.sleep();
        }

        // After connection are guaranteed to be established, we can publish messages now.
        theta[i].data = positions[i];
        cout << " " << positions[i];
        publishers[i].publish(theta[i]);
        ros::spinOnce();
    }  
    cout << " ]" << GREEN << " [ DONE ] " << NC << endl;  
}

/**
 * @brief utility function that prints all the information about robot position
 */
void print_robot_status()
{
    cout << RED << "Robot informations " << NC << endl;
    print_joints(jointState);
    pair<Vector3ld, Matrix3ld> forward = computeForwardKinematics(jointState);
    print_position(forward.first(0), forward.first(1), forward.first(2));
    print_eluler_angles(matrix2euler(forward.second));
    cout << endl << "----------------------------------------------------------" << endl << endl;
}

static void set_subscribers(ros::NodeHandle n)
{
    subscribers[0] = n.subscribe("/shoulder_pan_joint_position_controller/state", queue_size, get_position_shoulder_pan);
    subscribers[1] = n.subscribe("/shoulder_lift_joint_position_controller/state", queue_size, get_position_shoulder_lift);
    subscribers[2] = n.subscribe("/elbow_joint_position_controller/state", queue_size, get_position_elbow);
    subscribers[3] = n.subscribe("/wrist_1_joint_position_controller/state", queue_size, get_position_wrist_1);
    subscribers[4] = n.subscribe("/wrist_2_joint_position_controller/state", queue_size, get_position_wrist_2);
    subscribers[5] = n.subscribe("/wrist_3_joint_position_controller/state", queue_size, get_position_wrist_3);
}

static void set_publishers(ros::NodeHandle n)
{
    publishers[0] = n.advertise<std_msgs::Float64>("/shoulder_pan_joint_position_controller/command", queue_size);
	publishers[1] = n.advertise<std_msgs::Float64>("/shoulder_lift_joint_position_controller/command", queue_size);
	publishers[2] = n.advertise<std_msgs::Float64>("/elbow_joint_position_controller/command", queue_size);
	publishers[3] = n.advertise<std_msgs::Float64>("/wrist_1_joint_position_controller/command", queue_size);
	publishers[4] = n.advertise<std_msgs::Float64>("/wrist_2_joint_position_controller/command", queue_size);
	publishers[5] = n.advertise<std_msgs::Float64>("/wrist_3_joint_position_controller/command", queue_size);
}

static void get_position_shoulder_pan(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {jointState[0] = ctr_msg->process_value;}
static void get_position_shoulder_lift(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {jointState[1] = ctr_msg->process_value;}
static void get_position_elbow(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {jointState[2] = ctr_msg->process_value;}
static void get_position_wrist_1(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {jointState[3] = ctr_msg->process_value;}
static void get_position_wrist_2(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {jointState[4] = ctr_msg->process_value;}
static void get_position_wrist_3(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {jointState[5] = ctr_msg->process_value;}