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
    cout << BLUE << "Eluler angles (roll pitch yaw): " << NC << "[ ";
    cout << YELLOW << ea(0) << NC << ", ";
    cout << YELLOW << ea(1) << NC << ", ";
    cout << YELLOW << ea(2) << NC << " ]" << endl;
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
    print_eluler_angles(radToDeg(matrixToEuler(forward.second)));
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

void gripper_set(long double gripperValueToSet)
{
  control_msgs::GripperCommandActionGoal gripperValue;
  gripperValue.goal.command.position = gripperValueToSet;
  
  gripperPublisher.publish(gripperValue);
}

void set_subscribers(ros::NodeHandle n)
{
    subscribers[0] = n.subscribe("/shoulder_pan_joint_position_controller/state", queue_size, get_position_shoulder_pan);
    subscribers[1] = n.subscribe("/shoulder_lift_joint_position_controller/state", queue_size, get_position_shoulder_lift);
    subscribers[2] = n.subscribe("/elbow_joint_position_controller/state", queue_size, get_position_elbow);
    subscribers[3] = n.subscribe("/wrist_1_joint_position_controller/state", queue_size, get_position_wrist_1);
    subscribers[4] = n.subscribe("/wrist_2_joint_position_controller/state", queue_size, get_position_wrist_2);
    subscribers[5] = n.subscribe("/wrist_3_joint_position_controller/state", queue_size, get_position_wrist_3);
    //gripperSubscriber = n.subscribe("", queue_size, );
}

void set_publishers(ros::NodeHandle n)
{
    publishers[0] = n.advertise<std_msgs::Float64>("/shoulder_pan_joint_position_controller/command", queue_size);
	publishers[1] = n.advertise<std_msgs::Float64>("/shoulder_lift_joint_position_controller/command", queue_size);
	publishers[2] = n.advertise<std_msgs::Float64>("/elbow_joint_position_controller/command", queue_size);
	publishers[3] = n.advertise<std_msgs::Float64>("/wrist_1_joint_position_controller/command", queue_size);
	publishers[4] = n.advertise<std_msgs::Float64>("/wrist_2_joint_position_controller/command", queue_size);
	publishers[5] = n.advertise<std_msgs::Float64>("/wrist_3_joint_position_controller/command", queue_size);
    //gripperPublisher = n.advertise<std_msgs::Float64>("", queue_size);
}

void get_position_shoulder_pan(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {jointState[0] = ctr_msg->process_value;}
void get_position_shoulder_lift(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {jointState[1] = ctr_msg->process_value;}
void get_position_elbow(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {jointState[2] = ctr_msg->process_value;}
void get_position_wrist_1(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {jointState[3] = ctr_msg->process_value;}
void get_position_wrist_2(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {jointState[4] = ctr_msg->process_value;}
void get_position_wrist_3(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {jointState[5] = ctr_msg->process_value;}
