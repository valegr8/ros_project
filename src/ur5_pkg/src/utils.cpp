#include <ur5_pkg/utils.h>

void print_position(long double x, long double y, long double z)
{
    cout << BLUE << "Position ( x y z ) : " << NC << "[ ";
    cout << YELLOW << x << NC << ", "; 
    cout << YELLOW << y << NC << ", ";
    cout << YELLOW << z << NC << " ] ";
    cout << endl;
}

void print_eluler_angles(Vector3ld ea)
{
    cout << BLUE << "Eluler angles - Roll(z) Pitch(y) Yaw(x): " << NC << "[ ";
    cout << ea(0) << ", ";
    cout << ea(1) << ", ";
    cout << ea(2) << " ]" << endl;
}

void print_joints(vector<long double> joints)
{
    cout << BLUE << "Joints: " << NC << " [ ";
    for(auto it : joints)
        cout << it << " ";
    cout << "]" << endl;
}

void print_desidered_joints(Vector6ld joints)
{
    cout << BLUE << "Desired joint values: " << NC << " [ ";
    for(auto it : joints)
        cout << it << " ";
    cout << "]" << endl;
}

void print_robot_status()
{
    cout << RED << "Robot informations " << NC << endl;
    print_joints(jointState);
    pair<Vector3ld, Matrix3ld> forward = computeForwardKinematics(jointState);
    print_position(forward.first(0), forward.first(1), forward.first(2));
    print_eluler_angles(matrix2euler(forward.second));
    cout << endl << "----------------------------------------------------------" << endl << endl;
}

Matrix3ld euler2matrix(long double roll, long double pitch, long double yaw)
{
    AngleAxis<long double> rollAngle(roll, Vector3ld::UnitZ());
    AngleAxis<long double> yawAngle(yaw, Vector3ld::UnitY());
    AngleAxis<long double> pitchAngle(pitch, Vector3ld::UnitX());

    Eigen::Quaternion<long double> q = rollAngle * yawAngle * pitchAngle;

    Matrix3ld rotationMatrix = q.matrix();

    return rotationMatrix;
}

Vector3ld matrix2euler(Matrix3ld m) 
{
    return m.eulerAngles(0, 1, 2);
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

void set_subscribers(ros::NodeHandle n)
{
    subscribers[0] = n.subscribe("/shoulder_pan_joint_position_controller/state", queue_size, get_position_shoulder_pan);
    subscribers[1] = n.subscribe("/shoulder_lift_joint_position_controller/state", queue_size, get_position_shoulder_lift);
    subscribers[2] = n.subscribe("/elbow_joint_position_controller/state", queue_size, get_position_elbow);
    subscribers[3] = n.subscribe("/wrist_1_joint_position_controller/state", queue_size, get_position_wrist_1);
    subscribers[4] = n.subscribe("/wrist_2_joint_position_controller/state", queue_size, get_position_wrist_2);
    subscribers[5] = n.subscribe("/wrist_3_joint_position_controller/state", queue_size, get_position_wrist_3);
}

void set_publishers(ros::NodeHandle n)
{
    publishers[0] = n.advertise<std_msgs::Float64>("/shoulder_pan_joint_position_controller/command", queue_size);
	publishers[1] = n.advertise<std_msgs::Float64>("/shoulder_lift_joint_position_controller/command", queue_size);
	publishers[2] = n.advertise<std_msgs::Float64>("/elbow_joint_position_controller/command", queue_size);
	publishers[3] = n.advertise<std_msgs::Float64>("/wrist_1_joint_position_controller/command", queue_size);
	publishers[4] = n.advertise<std_msgs::Float64>("/wrist_2_joint_position_controller/command", queue_size);
	publishers[5] = n.advertise<std_msgs::Float64>("/wrist_3_joint_position_controller/command", queue_size);
}

void get_position_shoulder_pan(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {jointState[0] = ctr_msg->process_value;}
void get_position_shoulder_lift(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {jointState[1] = ctr_msg->process_value;}
void get_position_elbow(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {jointState[2] = ctr_msg->process_value;}
void get_position_wrist_1(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {jointState[3] = ctr_msg->process_value;}
void get_position_wrist_2(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {jointState[4] = ctr_msg->process_value;}
void get_position_wrist_3(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {jointState[5] = ctr_msg->process_value;}
