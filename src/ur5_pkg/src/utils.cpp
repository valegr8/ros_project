#include <ur5_pkg/utils.h>

void signal_handler(int signal)
{
    //cout <<
    //gSignalStatus = signal;
}

void set_signals()
{
    // Install a signal handler
    std::signal(SIGINT, signal_handler);
} 

void print_position(long double x, long double y, long double z)
{
    cout << BLUE << "Position (x y z): " << NC << "[ ";
    cout << YELLOW << x << NC << ", "; 
    cout << YELLOW << y << NC << ", ";
    cout << YELLOW << z << NC << " ] ";
    cout << endl;
}

void print_eluler_angles(Vector3ld ea)
{
    cout << BLUE << "Eluler angles (roll pitch yaw): " << NC << "[ ";
    cout << YELLOW << ea(0) << NC << ", ";
    cout << YELLOW << ea(1) << NC << ", ";
    cout << YELLOW << ea(2) << NC << " ]" << endl;
}

void print_joints(vector<long double> joints)
{
    cout << BLUE << "Joints: " << NC << " [ ";
    cout << YELLOW << joints[0] << NC << ", ";
    cout << YELLOW << joints[1] << NC << ", ";
    cout << YELLOW << joints[2] << NC << ", ";
    cout << YELLOW << joints[3] << NC << ", ";
    cout << YELLOW << joints[4] << NC << ", ";
    cout << YELLOW << joints[5] << NC << " ";
    cout << "]" << endl;
}

void print_desidered_joints(Vector6ld joints)
{
    cout << BLUE << "Desired joint values: " << NC << " [ ";
    for(auto it : joints)
        cout << it << " ";
    cout << "]" << endl;
}

void print_gripper_status()
{
    cout << BLUE << "Gripper state: " << NC << " [ " << YELLOW << gripperState << NC << " ]" << endl;
}

void print_robot_status()
{
    cout << RED << "Robot informations " << NC << endl;
    print_joints(jointState);
    pair<Vector3ld, Matrix3ld> forward = computeForwardKinematics(jointState);
    print_position(forward.first(0), forward.first(1), forward.first(2));
    print_eluler_angles(radToDeg(matrixToEuler(forward.second)));
    if(gripper)
        print_gripper_status();
    cout << endl << "----------------------------------------------------------" << endl << endl;
}

Vector3ld radToDeg(Vector3ld in)
{
    long double mul = (180.0/PI);
    return Vector3ld{in(0)*mul, in(1)*mul, in(2)*mul};
}

Vector3ld degToRad(Vector3ld in)
{
    long double mul = (PI/180.0);
    return Vector3ld{in(0)*mul, in(1)*mul, in(2)*mul};
}

Matrix3ld eulerToMatrix(Vector3ld eulerAngles)
{
    AngleAxis<long double> rollAngle(eulerAngles(0), Vector3ld::UnitX());
    AngleAxis<long double> pitchAngle(eulerAngles(1), Vector3ld::UnitY());
    AngleAxis<long double> yawAngle(eulerAngles(2), Vector3ld::UnitZ());
    Eigen::Quaternion<long double> q = rollAngle * pitchAngle * yawAngle;
    return q.matrix();
}

Vector3ld matrixToEuler(Matrix3ld rotationMatrix) 
{
    return rotationMatrix.eulerAngles(0, 1, 2);
}

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

void gripper_set(long double gripperValueToSet, ros::Rate& loop_rate)
{
    //Mandatory to public
    std_msgs::Float64 gripperValue;

    //Putting the value into the std_msgs::Float64 structure
    gripperValue.data = gripperValueToSet;

    //Public
    gripperPublisher.publish(gripperValue);

    //The gripper takes time to set
    loop_rate.sleep();
    loop_rate.sleep();
    loop_rate.sleep();
    ros::spinOnce();
}

bool createDynamicLink(ros::NodeHandle nodeHandle, string name1, string name2, string link1, string link2)
{
    //Loading the client for the plugin
    ros::ServiceClient client = nodeHandle.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");

    //Create dynamic link
    return dynamicLinkAction(client, name1, name2, link1, link2);
}

bool destroyDynamicLink(ros::NodeHandle nodeHandle, string name1, string name2, string link1, string link2)
{
    //Loading the client for the plugin
    ros::ServiceClient client = nodeHandle.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");

    //Destroy dynamic link
    return dynamicLinkAction(client, name1, name2, link1, link2);
}

bool dynamicLinkAction(ros::ServiceClient client, string name1, string name2, string link1, string link2)
{
    //Creating request and response
    gazebo_ros_link_attacher::AttachRequest req;
    gazebo_ros_link_attacher::AttachResponse res;

    //Filling the request
    req.model_name_1 = name1;
    req.model_name_2 = name2;
    req.link_name_1 = link1;
    req.link_name_2 = link2;

    client.call(req, res);

    return res.ok;
}

void set_subscribers(ros::NodeHandle n)
{
    subscribers[0] = n.subscribe("/shoulder_pan_joint_position_controller/state", queue_size, get_position_shoulder_pan);
    subscribers[1] = n.subscribe("/shoulder_lift_joint_position_controller/state", queue_size, get_position_shoulder_lift);
    subscribers[2] = n.subscribe("/elbow_joint_position_controller/state", queue_size, get_position_elbow);
    subscribers[3] = n.subscribe("/wrist_1_joint_position_controller/state", queue_size, get_position_wrist_1);
    subscribers[4] = n.subscribe("/wrist_2_joint_position_controller/state", queue_size, get_position_wrist_2);
    subscribers[5] = n.subscribe("/wrist_3_joint_position_controller/state", queue_size, get_position_wrist_3);
    if(gripper)
        gripperSubscriber = n.subscribe("/gripper_controller/state", queue_size, get_position_gripper);
}

void set_publishers(ros::NodeHandle n)
{
    publishers[0] = n.advertise<std_msgs::Float64>("/shoulder_pan_joint_position_controller/command", queue_size);
	publishers[1] = n.advertise<std_msgs::Float64>("/shoulder_lift_joint_position_controller/command", queue_size);
	publishers[2] = n.advertise<std_msgs::Float64>("/elbow_joint_position_controller/command", queue_size);
	publishers[3] = n.advertise<std_msgs::Float64>("/wrist_1_joint_position_controller/command", queue_size);
	publishers[4] = n.advertise<std_msgs::Float64>("/wrist_2_joint_position_controller/command", queue_size);
	publishers[5] = n.advertise<std_msgs::Float64>("/wrist_3_joint_position_controller/command", queue_size);
    if(gripper)
        gripperPublisher = n.advertise<std_msgs::Float64>("/gripper_controller/command", queue_size);
}

void get_position_shoulder_pan(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {jointState[0] = ctr_msg->process_value;}
void get_position_shoulder_lift(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {jointState[1] = ctr_msg->process_value;}
void get_position_elbow(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {jointState[2] = ctr_msg->process_value;}
void get_position_wrist_1(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {jointState[3] = ctr_msg->process_value;}
void get_position_wrist_2(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {jointState[4] = ctr_msg->process_value;}
void get_position_wrist_3(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {jointState[5] = ctr_msg->process_value;}
void get_position_gripper(const control_msgs::JointControllerState::ConstPtr& ctr_msg) { gripperState = ctr_msg->process_value;}
