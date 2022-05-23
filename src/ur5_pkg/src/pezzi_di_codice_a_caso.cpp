
/*
void testGripperRotation(ros::Rate& loop_rate)
{
    //Chiudo il gripper
    gripper_set(1, loop_rate);

    long double val;

    while(true)
    {
        cout << "Inserici valore: ";
        cin >> val;

        set_joint_values({0, -1.5, 1, 0, 0, val});

        loop_rate.sleep();
        loop_rate.sleep();
        loop_rate.sleep();
        loop_rate.sleep();
        loop_rate.sleep();
        ros::spinOnce();
        ros::spinOnce();
        ros::spinOnce();
        ros::spinOnce();
        ros::spinOnce();

    }

    /*
    for(long double para = -2*PI; para < 2*PI; para+=0.1)
    {
        set_joint_values({0, -1.5, 1, 0, 0, para});
        loop_rate.sleep();
        loop_rate.sleep();
        loop_rate.sleep();
        loop_rate.sleep();
        loop_rate.sleep();
        ros::spinOnce();
        ros::spinOnce();
        ros::spinOnce();
        ros::spinOnce();
        ros::spinOnce();
    }*/
        

    /*
    int i;
    while(true)
    {
        loop_rate.sleep();
        ros::spinOnce();
        print_robot_status();
        cout << "continua ... ";
        cin >> i;

        

        loop_rate.sleep();
        ros::spinOnce();
        print_robot_status();
        cout << "continua ... ";
        cin >> i;

        set_joint_values({0, -1.5, 1, 0, 0, -PI});

        loop_rate.sleep();
        ros::spinOnce();
        print_robot_status();
        cout << "continua ... ";
        cin >> i;

        set_joint_values({0, -1.5, 1, 0, 0, PI});
        
    }
    */
//}

/*
void testGripper(ros::Rate& loop_rate)
{
    std_msgs::Float64 gripperValue;
    long double value;

    while(true)
    {
        loop_rate.sleep();
        loop_rate.sleep();
        loop_rate.sleep();
        //loop_rate.sleep();
        //ros::spinOnce();
        //ros::spinOnce();
        //ros::spinOnce();
        ros::spinOnce();
        
        //Stampo info robot
        print_robot_status();

        cout << "Inserisci valore del gripper: ";
        cin >> value;

        //Copio il valore in std_msgs::Float64
        gripperValue.data = value;

        //Pubblico
        gripperPublisher.publish(gripperValue);
    }
}
*/