#include <ur5_pkg/utils.h>

//Variabili globali
vector<ros::Subscriber> subscribers(JOINT_NUM); //< global subscribers vector
vector<ros::Publisher> publishers(JOINT_NUM);  // global publisher vector
ros::Subscriber gripperSubscriber; // gripper subscriber
ros::Publisher gripperPublisher; 
vector<long double> jointState(JOINT_NUM); // contains all /state values of the joints
long double gripperState; // contains state values of the gripper
int queue_size; // used for publisher and subscribers queue size 
bool debug = true; //For debug

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

    ros::spinOnce();
    loop_rate.sleep();

    cout << GREEN << " [ DONE ] " << endl;

    //just the first time, to set the initial position of the robot
    if(ros::ok())
    {
        jointState = {0, -1.5, 1, 0, 0, -1.5};
        set_joint_values(jointState);
    
        ros::spinOnce();
        loop_rate.sleep();
    }

    cout << endl << "----------------------------------------------------------" << endl << endl;

    //Modalità in cui inserisci i valori e vai al punto inserito
    askUserGoToPoint(loop_rate);

    //Modalità in cui il robot prende il cubo centrale
    //pigliaCuboCentrale(loop_rate);

    //ROS_ERROR("Ros not working\n");
    return 1; 
}

void pigliaCuboCentrale(ros::Rate& loop_rate)
{
    cout << BLUE << "***  MODALITA' PIGLIA CUBO CENTRALE  ***" << NC << endl << endl;

    ros::spinOnce();
    loop_rate.sleep();

    //Il cubo centrale è a x:0 y:0.5, z < 0.25
    //End effector punato in giù 
    //Roll : 180°, pitch: 0°, yaw:0°

    cout << "Mi posizione sopra il cubo 0.0, 0.5, 0.4" << endl;
    //Mi posizione sopra il cubo
    pointToPointMotionPlan(make_pair(Vector3ld{0.0, 0.5, 0.4}, degToRad(Vector3ld{180.0, 0.0, 90.0})), loop_rate, 0.0, 1.0, 0.01);

    cout << "Apro gripper" << endl;
    //Apro il gripper
    gripper_set(0, loop_rate);

    print_robot_status();
    
    cout << "Prendo il cubo 0.0, 0.5, 0.3" << endl;
    //Prendo il cubo
    pointToPointMotionPlan(make_pair(Vector3ld{0.0, 0.5, 0.3}, degToRad(Vector3ld{180.0, 0.0, 90.0})), loop_rate, 0.0, 1.0, 0.01);
    
    cout << "Chudo gripper" << endl;
    //Chiudo il gripper
    gripper_set(1, loop_rate);

    print_robot_status();

    cout << "Alzo il cubo 0.0, 0.5, 0.4" << endl;
    //Lo alzo
    pointToPointMotionPlan(make_pair(Vector3ld{0.0, 0.5, 0.4}, degToRad(Vector3ld{180.0, 0.0, 0.0})), loop_rate, 0.0, 1.0, 0.01);

    print_robot_status();

    cout << "Mi sposto fuori dal tavolo 0.5, 0.5, 0.2" << endl;
    //Mi sposto fuori dal tavolo
    pointToPointMotionPlan(make_pair(Vector3ld{0.5, 0.5, 0.2}, degToRad(Vector3ld{180.0, 0.0, 0.0})), loop_rate, 0.0, 1.0, 0.01);

    cout << "Apro gripper" << endl;
    //Apro il gripper
    gripper_set(0, loop_rate);

    print_robot_status();
}

void askUserGoToPoint(ros::Rate& loop_rate)
{
    string str; //Usata per leggere
    pair<Vector3ld, Vector3ld> to; //Per la posizione finale
    
    cout << BLUE << "***  MODALITA' MOVIMENTO GUIDATO  ***" << NC << endl << endl;

    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();

        //Stampo informazioni
        print_robot_status();

        {
            cout << "Vanno bene gli attuali valori di x, y e z? (y/n) : ";
            cin >> str;
            if(str == "y")
                //Prendo i valori attuali
                to.first = computeForwardKinematics(jointState).first;
            else
            {
                cout << "Inserisci x: ";
                cin >> to.first(0);
                cout << "Inserisci y: ";
                cin >> to.first(1);
                cout << "Inserisci z: ";
                cin >> to.first(2);
            }
            cout << "Vanno bene gli attuali valori di roll, pitch e yaw? (y/n) : ";
            cin >> str;
            if(str == "y")
            {
                //Prendo i valori attuali convertiti in gradi
                to.second = radToDeg(matrixToEuler(computeForwardKinematics(jointState).second));
            }
            else
            {
                cout << "Inserisci roll in gradi: ";
                cin >> to.second(0);
                cout << "Inserisci pitch in gradi: ";
                cin >> to.second(1);
                cout << "Inserisci yaw in gradi: ";
                cin >> to.second(2);
            }
        }

        cout << BLUE << "Vado a " << NC << "[ " << to.first(0) << ", " << to.first(1) << ", " << to.first(2) << " ] "
        << BLUE << " con rotazione: " << NC << " [ " << to.second(0) << ", " << to.second(1) << ", " << to.second(2) << " ] " << endl;

        //Converione gradi radianti
        to.second = degToRad(to.second);

        if(pointToPointMotionPlan(to, loop_rate, 0.0, 1.0, 0.01))
            cout << GREEN << "[ DONE ]" << NC << endl;
        else
            cout << RED << "[ FAILED ]" << NC << endl;

        cout << endl << "----------------------------------------------------------" << endl << endl;

        sleep(1);
        loop_rate.sleep();
    }
}
