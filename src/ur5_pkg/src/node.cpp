#include <ur5_pkg/utils.h>
#include <ur5_pkg/json.hpp>
using json = nlohmann::json;

//Variabili globali
vector<ros::Subscriber> subscribers(JOINT_NUM); //< global subscribers vector
vector<ros::Publisher> publishers(JOINT_NUM);  // global publisher vector
ros::Subscriber gripperSubscriber; // gripper subscriber
ros::Publisher gripperPublisher; 
vector<long double> jointState(JOINT_NUM); // contains all /state values of the joints
long double gripperState; // contains state values of the gripper
int queue_size; // used for publisher and subscribers queue size 
bool debug = true; //For debug
bool gripper = true; //For gripper present
json posData;

void moveLegoToPos(ros::Rate& loop_rate,ros::NodeHandle nodeHandle);
void moveLegosToPos(ros::Rate& loop_rate,ros::NodeHandle nodeHandle);

int main (int argc, char **argv)
{
    if(debug)
        cout << GREEN << "Debug attivo" << NC << endl << endl;
    else
        cout << RED << "Debug non attivo" << NC << endl << endl;

    cout << BLUE << "Starting ROS node..." << NC;

    //Setting all signals handler
    void set_signals();

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
        jointState = {0, -1.5, 1, 0, 0, 0};
        set_joint_values(jointState);
    
        ros::spinOnce();
        loop_rate.sleep();
    }

    cout << endl << "----------------------------------------------------------" << endl << endl;

    

    moveLegosToPos(loop_rate,nodeHandle);
    //Modalità in cui inserisci i valori e vai al punto inserito
    //askUserGoToPoint(loop_rate);
    //Modalità in cui il robot prende i cubi e li impila
    // pigliaCuboCentrale(loop_rate, nodeHandle);

    //ROS_ERROR("Ros not working\n");
    return 1; 
}


bool ok =  true;

void getData(const std_msgs::String::ConstPtr& msg){
    try{
        if(ok){
            posData = json::parse(msg->data.c_str());
            ok= false;
        }

    }catch (int e){}
   
}

void moveLegosToPos(ros::Rate& loop_rate,ros::NodeHandle nodeHandle){
    string str; //Usata per leggere
    pair<Vector3ld, Vector3ld> to; //Per la posizione finale
    long double gripperValue = 0;

    // cout << BLUE << "***  MODALITA' SMART  ***" << NC << endl << endl;
    cout << GREEN << "Getting position data..." << NC << endl << endl;
    ros::Rate loop_rateDetection(1);

    ros::Subscriber nVis = nodeHandle.subscribe("objDetect",2, getData);
    ros::spinOnce();
    mySleep(loop_rateDetection);
    
    cout << posData.dump(2)<< endl;
    for(int i = 0; i < posData["obj"].size(); i++){
        string name = posData["obj"][i]["name"];
        float x = posData["obj"][i]["x"];
        float y = posData["obj"][i]["y"];


        //prendo il primo blocco lego
        cout << "Mi posiziono sopra "<< name  << " -> x: "<< x << " y: "<< y << endl;
        pointToPointMotionPlan(make_pair(Vector3ld{x, y, 0.5}, degToRad(Vector3ld{180.0, 0.0, 45.0})), loop_rate, 0.0, 1.0, 0.01);
        mySleep(loop_rate);

        cout << "Apro gripper 0.0" << endl;
        gripper_set(0.0, loop_rate);

        cout << "Prendo "<< name  << " -> x: "<< x << " y: "<< y << endl;

        // x y z of the end effector
        Vector3ld end_effector = computeForwardKinematics(jointState).first;
        cout << "x: " << end_effector(0) << " - y: "  << end_effector(1) << " - z: " <<end_effector(2) << endl;
        pointToPointMotionPlan(make_pair(Vector3ld{x, y, 0.25}, degToRad(Vector3ld{180.0, 0.0, 45.0})), loop_rate, 0.0, 1.0, 0.01);
        mySleep(loop_rate);

        cout << "Chudo gripper {0.20}" << endl;
        gripper_set(0.20, loop_rate);

        // cout << "Creo link dinamico robot - " << name<< endl;
        createDynamicLink(nodeHandle, "robot" , name, "wrist_3_link", "link");

        cout << "Alzo  "<< name  << " -> x: "<< x << " y: "<< y << endl;
        pointToPointMotionPlan(make_pair(Vector3ld{x, y, 0.5}, degToRad(Vector3ld{180.0, 0.0, 45.0})), loop_rate, 0.0, 1.0, 0.01);
        mySleep(loop_rate);

        mySleep(loop_rate);
        print_robot_status();

        // cout << "Inserisci x: ";
        // cin >> to.first(0);
        // cout << "Inserisci y: ";
        // cin >> to.first(1);
        // cout << "Inserisci z: ";
        // cin >> to.first(2);

        //posizione finale
        if (name.compare("X1-Y1-Z2") == 0)
        {
            to.first(0) = -0.45;
            to.first(1) = 0;
        }
        if (name.compare("X1-Y2-Z1") == 0) {
            to.first(0) = -0.45;
            to.first(1) = 0.1;
        }
        if (name.compare("X1-Y2-Z2-CHAMFER") == 0) {
            to.first(0) = -0.45;
            to.first(1) = 0.5;
        }
        if (name.compare("X1-Y2-Z2-TWINFILLET") == 0) {
            to.first(0) = 0.45;
            to.first(1) = 0;
        }
        if (name.compare("X1-Y2-Z2") == 0) {
            to.first(0) = 0.45;
            to.first(1) = 0.1;
        }
        if (name.compare("X1-Y3-Z2-FILLET") == 0) {
            to.first(0) = 0.45;
            to.first(1) = 0.2;
        }
        if (name.compare("X1-Y3-Z2") == 0) {
            to.first(0) = 0.45;
            to.first(1) = 0.3;
        }
        if (name.compare("X1-Y4-Z1") == 0) {
            to.first(0) = 0.45;
            to.first(1) = 0.4;
        }
        if (name.compare("X1-Y4-Z2") == 0) {
            to.first(0) = -0.45;
            to.first(1) = 0.2;
        }
        if (name.compare("X2-Y2-Z2-FILLET") == 0) {
            to.first(0) = -0.45;
            to.first(1) = 0.3;
        }
        if (name.compare("X2-Y2-Z2") == 0) {
            to.first(0) = -0.45;
            to.first(1) = 0.4;
        }

        to.first(2) = 0.25;
        
        cout << RED << name << " va nella posiione: x " << to.first(0) << " - y " <<  to.first(2) << " - z " << to.first(2) << NC << endl;

        cout << "Mi posizione sopra la posizione scelta" << endl;
        pointToPointMotionPlan(make_pair(Vector3ld{to.first(0), to.first(1), to.first(2)}, degToRad(Vector3ld{180.0, 0.0, 45.0})), loop_rate, 0.0, 1.0, 0.01);
        mySleep(loop_rate);

        cout << "Apro gripper {0.0}" << endl;
        gripper_set(0.0, loop_rate);

        // cout << "Distruggo link dinamico wrist_3_joint - " << name << endl;
        destroyDynamicLink(nodeHandle, "robot" , name, "wrist_3_link", "link");

        mySleep(loop_rate);
        print_robot_status();
    }
}

void moveLegoToPos(ros::Rate& loop_rate,ros::NodeHandle nodeHandle){
    string str; //Usata per leggere
    pair<Vector3ld, Vector3ld> to; //Per la posizione finale
    long double gripperValue = 0;
    cout << BLUE << "***  MODALITA' SMART  ***" << NC << endl << endl;
    cout << GREEN << "Getting position data..." << NC << endl << endl;
    ros::Rate loop_rateDetection(1);

    ros::Subscriber nVis = nodeHandle.subscribe("objDetect",2, getData);
    ros::spinOnce();
    mySleep(loop_rateDetection);
    
    cout << posData.dump(2)<< endl;
    string name = posData["obj"][0]["name"];
    float x = posData["obj"][0]["x"];
    float y = posData["obj"][0]["y"];


    //prendo il primo blocco lego
    cout << "Mi posizione sopra "<< name  << " -> x: "<< x << " y: "<< y << endl;
    pointToPointMotionPlan(make_pair(Vector3ld{x, y, 0.5}, degToRad(Vector3ld{180.0, 0.0, 45.0})), loop_rate, 0.0, 1.0, 0.01);
    mySleep(loop_rate);

    cout << "Apro gripper 0.0" << endl;
    gripper_set(0.0, loop_rate);

    cout << "Prendo "<< name  << " -> x: "<< x << " y: "<< y << endl;

    // x y z of the end effector
    Vector3ld end_effector = computeForwardKinematics(jointState).first;
    cout << "x: " << end_effector(0) << " - y: "  << end_effector(1) << " - z: " <<end_effector(2) << endl;
    pointToPointMotionPlan(make_pair(Vector3ld{x, y, 0.25}, degToRad(Vector3ld{180.0, 0.0, 45.0})), loop_rate, 0.0, 1.0, 0.01);
    mySleep(loop_rate);

    cout << "Chudo gripper {0.20}" << endl;
    gripper_set(0.20, loop_rate);

    // cout << "Creo link dinamico robot - " << name<< endl;
    createDynamicLink(nodeHandle, "robot" , name, "wrist_3_link", "link");

    cout << "Alzo  "<< name  << " -> x: "<< x << " y: "<< y << endl;
    pointToPointMotionPlan(make_pair(Vector3ld{x, y, 0.5}, degToRad(Vector3ld{180.0, 0.0, 45.0})), loop_rate, 0.0, 1.0, 0.01);
    mySleep(loop_rate);

    mySleep(loop_rate);
    print_robot_status();

    // cout << "Inserisci x: ";
    // cin >> to.first(0);
    // cout << "Inserisci y: ";
    // cin >> to.first(1);
    // cout << "Inserisci z: ";
    // cin >> to.first(2);

    //posizione finale
    // if (name == "")
    to.first(0) = 0.2;
    to.first(1) = 0.3;
    to.first(2) = 0.25;

    cout << "Mi posizione sopra la posizione scelta" << endl;
    pointToPointMotionPlan(make_pair(Vector3ld{to.first(0), to.first(1), to.first(2)}, degToRad(Vector3ld{180.0, 0.0, 45.0})), loop_rate, 0.0, 1.0, 0.01);
    mySleep(loop_rate);

    cout << "Apro gripper {0.0}" << endl;
    gripper_set(0.0, loop_rate);

    // cout << "Distruggo link dinamico wrist_3_joint - " << name << endl;
    destroyDynamicLink(nodeHandle, "robot" , name, "wrist_3_link", "link");

    mySleep(loop_rate);
    print_robot_status();
}

// void pigliaCuboCentrale(ros::Rate& loop_rate, ros::NodeHandle nodeHandle)
// {
//     cout << BLUE << "***  MODALITA' PIGLIA CUBO CENTRALE  ***" << NC << endl << endl;
//     mySleep(loop_rate);

//     //Nozioni
//     //Il cubo centrale è a x:0 y:0.5, z < 0.25
//     //Il cubo a sinistra è a x:-0.16 y:0.5 z < 0.25
//     //Il cubo a destra è a x:0.16 y:0.5 z < 0.25
//     //End effector punato in giù : Roll : 180°, pitch: 0°, yaw:0°
//     //N.B. Roll, Pith, Yaw non sono univoci
//     //Un'altra possibile opzioni per puntare in giù l'end effector è Roll : 0°, pitch: 180°, yaw: 180°
//     //Questo link sotto è un simulatore, non utilizza i nostri stessi riferimenti ma fa capire il concetto
//     //https://compsci290-s2016.github.io/CoursePage/Materials/EulerAnglesViz/
//     //Yaw a 45° è l'angolo giusto per prendere un cubo
//     //Il robot è relativamente preciso ma, quando si devono fare certe distanze conviene coprire la maggiorparte della distanza
//     //con un primo movimento (es. posizionarsi sopra il cubo) e con un secondo movimento molto più breve (quindi più preciso)
//     //afferrare l'oggetto.

//     //Prendo cubo 1
//     {
//         cout << "Mi posizione sopra il cubo 1 {0.0, 0.5, 0.4} {180.0, 0.0, 45.0}" << endl;
//         pointToPointMotionPlan(make_pair(Vector3ld{0.0, 0.5, 0.4}, degToRad(Vector3ld{180.0, 0.0, 45.0})), loop_rate, 0.0, 1.0, 0.01);
//         mySleep(loop_rate);

//         cout << "Apro gripper 0.0" << endl;
//         gripper_set(0.0, loop_rate);

//         cout << "Prendo il cubo {0.0, 0.5, 0.175} {180.0, 0.0, 45.0}" << endl;
//         pointToPointMotionPlan(make_pair(Vector3ld{0.0, 0.5, 0.175}, degToRad(Vector3ld{180.0, 0.0, 45.0})), loop_rate, 0.0, 1.0, 0.01);
//         mySleep(loop_rate);

//         cout << "Chudo gripper {0.20}" << endl;
//         gripper_set(0.20, loop_rate);

//         cout << "Creo link dinamico robot - cube1" << endl;
//         createDynamicLink(nodeHandle, "robot" , "cube1", "wrist_3_link", "link");

//         cout << "Alzo il cubo {0.0, 0.5, 0.4} {180.0, 0.0, 45.0}" << endl;
//         pointToPointMotionPlan(make_pair(Vector3ld{0.0, 0.5, 0.4}, degToRad(Vector3ld{180.0, 0.0, 45.0})), loop_rate, 0.0, 1.0, 0.01);
//         mySleep(loop_rate);

//         mySleep(loop_rate);
//         print_robot_status();
//     }

//     //Metto cubo 1 su cubo 3
//     {
//         cout << "Mi posizione sopra il cubo 3 {-0.16, 0.5, 0.4} {180.0, 0.0, 45.0}" << endl;
//         pointToPointMotionPlan(make_pair(Vector3ld{-0.16, 0.5, 0.4}, degToRad(Vector3ld{180.0, 0.0, 45.0})), loop_rate, 0.0, 1.0, 0.01);
//         mySleep(loop_rate);

//         cout << "Impilo il cubo {-0.16, 0.5, 0.225} {180.0, 0.0, 45.0}" << endl;
//         pointToPointMotionPlan(make_pair(Vector3ld{-0.16, 0.5, 0.225}, degToRad(Vector3ld{180.0, 0.0, 45.0})), loop_rate, 0.0, 1.0, 0.01);
//         mySleep(loop_rate);

//         cout << "Distruggo link dinamico wrist_3_joint - cube1" << endl;
//         destroyDynamicLink(nodeHandle, "robot" , "cube1", "wrist_3_link", "link");

//         cout << "Apro gripper {0.0}" << endl;
//         gripper_set(0.0, loop_rate);

//         mySleep(loop_rate);
//         print_robot_status();
//     }

//     //Prendo cubo 2
//     {
//         cout << "Mi posizione sopra il cubo 2 {0.16, 0.5, 0.4} {180.0, 0.0, 45.0}" << endl;
//         pointToPointMotionPlan(make_pair(Vector3ld{0.16, 0.5, 0.4}, degToRad(Vector3ld{180.0, 0.0, 45.0})), loop_rate, 0.0, 1.0, 0.01);
//         mySleep(loop_rate);

//         cout << "Apro gripper 0.0" << endl;
//         gripper_set(0.0, loop_rate);

//         cout << "Prendo il cubo {0.16, 0.5, 0.175} {180.0, 0.0, 45.0}" << endl;
//         pointToPointMotionPlan(make_pair(Vector3ld{0.16, 0.5, 0.175}, degToRad(Vector3ld{180.0, 0.0, 45.0})), loop_rate, 0.0, 1.0, 0.01);
//         mySleep(loop_rate);

//         cout << "Chudo gripper {0.20}" << endl;
//         gripper_set(0.20, loop_rate);

//         cout << "Creo link dinamico robot - cube2" << endl;
//         createDynamicLink(nodeHandle, "robot" , "cube2", "wrist_3_link", "link");

//         cout << "Alzo il cubo {0.16, 0.5, 0.4} {180.0, 0.0, 45.0}" << endl;
//         pointToPointMotionPlan(make_pair(Vector3ld{0.16, 0.5, 0.4}, degToRad(Vector3ld{180.0, 0.0, 45.0})), loop_rate, 0.0, 1.0, 0.01);
//         mySleep(loop_rate);

//         mySleep(loop_rate);
//         print_robot_status();
//     }


//     //Metto cubo 2 su cubo 1 che è su cubo 3
//     {
//         cout << "Mi posizione sopra il cubo 3 {-0.16, 0.5, 0.4} {180.0, 0.0, 45.0}" << endl;
//         pointToPointMotionPlan(make_pair(Vector3ld{-0.16, 0.5, 0.4}, degToRad(Vector3ld{180.0, 0.0, 45.0})), loop_rate, 0.0, 1.0, 0.01);
//         mySleep(loop_rate);

//         cout << "Impilo il cubo {-0.16, 0.5, 0.275} {180.0, 0.0, 45.0}" << endl;
//         pointToPointMotionPlan(make_pair(Vector3ld{-0.16, 0.5, 0.275}, degToRad(Vector3ld{180.0, 0.0, 45.0})), loop_rate, 0.0, 1.0, 0.01);
//         mySleep(loop_rate);

//         cout << "Distruggo link dinamico wrist_3_joint - cube2" << endl;
//         destroyDynamicLink(nodeHandle, "robot" , "cube2", "wrist_3_link", "link");

//         cout << "Apro gripper {0.0}" << endl;
//         gripper_set(0.0, loop_rate);

//         mySleep(loop_rate);
//         print_robot_status();
//     }
// }

void askUserGoToPoint(ros::Rate& loop_rate)
{
    string str; //Usata per leggere
    pair<Vector3ld, Vector3ld> to; //Per la posizione finale
    long double gripperValue = 0;
    
    cout << BLUE << "***  MODALITA' MOVIMENTO GUIDATO  ***" << NC << endl << endl;

    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
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
            
            cout << "Inserisci valore gripper: ";
            cin >> gripperValue;
        }

        cout << BLUE << "Vado a " << NC << "[ " << to.first(0) << ", " << to.first(1) << ", " << to.first(2) << " ] "
        << BLUE << " con rotazione: " << NC << " [ " << to.second(0) << ", " << to.second(1) << ", " << to.second(2) << " ] " << endl;
        cout << BLUE << "Apro/chiudo il gripper a " << NC << "[ " << gripperValue << " ]" << endl;

        //Converione gradi radianti
        to.second = degToRad(to.second);

        //Muovo il gripper
        gripper_set(gripperValue, loop_rate);

        //Muovo il robot
        if(pointToPointMotionPlan(to, loop_rate, 0.0, 1.0, 0.01))
            cout << GREEN << "[ DONE ]" << NC << endl;
        else
            cout << RED << "[ FAILED ]" << NC << endl;

        cout << endl << "----------------------------------------------------------" << endl << endl;

        sleep(1);
        loop_rate.sleep();
    }
}

void mySleep(ros::Rate& loop_rate)
{
    ros::spinOnce();
    loop_rate.sleep();
    ros::spinOnce();
    loop_rate.sleep();
    ros::spinOnce();
    loop_rate.sleep();
    ros::spinOnce();
    loop_rate.sleep();
}

