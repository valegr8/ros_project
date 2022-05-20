#include <ur5_pkg/utils.h>

//Variabili globali
vector<ros::Subscriber> subscribers(JOINT_NUM); //< global subscribers vector
vector<ros::Publisher> publishers(JOINT_NUM);  // global publisher vector
vector<long double> jointState(JOINT_NUM); // contains all /state values of the joints
int queue_size; // used for publisher and subscribers queue size 
bool debug = true;

int main (int argc, char **argv)
{
    if(debug)
        cout << GREEN << "Debug attivo" << NC << endl << endl;
    else
        cout << RED << "Debug non attivo" << NC << endl << endl;

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
