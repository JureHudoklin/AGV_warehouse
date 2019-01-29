#include <ros/ros.h>

#include <robotcontrol.h>

//Messages
#include <std_msgs/Int32.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Range.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <std_srvs/Empty.h>

//Publishers
ros::Publisher pub;


//Subscribers
ros::Subscriber cmd_vel_sub; //In cmd_vel twist messages about velocities are published
    /*Create a Subscriber object that will call the 'callback' 
    function each timelisten to the counter topic and will*/
           
//Services
/*
List services: rosservice list
Call service: rosservice call /my_service
*/
ros::ServiceServer motors_on; 
ros::ServiceServer motors_off; 

//Define used variable
bool motor_state;
double v_x, v_y, a_z;

bool motors_on_call(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    motor_state = true;
    return true;
}

bool motors_off_call(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    motor_state = false;
    return true;
}
void twist_callback(const geometry_msgs::Twist &twist) {		//Zakaj kle const. Zakaj argument v fn ce ga spodi ne dam
	v_x = twist.linear.x;
	v_y = twist.linear.y;
	a_z = twist.angular.z;
	ROS_INFO("nove_hitrosti");
}


int main(int argc, char** argv) {
    

    //Definer and run the node
	ros::init(argc, argv, "robot");
	ros::NodeHandle nh;

    //Start adverstisers, subscribers and services
    pub = nh.advertise<std_msgs::Int32>("jure_topic", 1000);
    //sub = nh.subscribe("tema_subscriptiona", 1000, counterCallback); 
	motors_on = nh.advertiseService("/motors_on", motors_on_call);
	motors_off = nh.advertiseService("/motors_off", motors_off_call);


	std_msgs::Int32 spremenljivka;
	spremenljivka.data = 0;
	ros::Rate loop_rate(2);



	//INITIALIZE MOTORS and ENCODERS
	/*
	Initializes the use of motors.
	Initializes the use of encoders.
	If initialization is unsucessfol returns error.
	*/
	if (use_motors == true)	{

		ROS_INFO("Initialize motors and encoders");

		cmd_vel_sub = nh.subscribe("/cmd_vel", 100, &twist_callback); 
		if(rc_motors_init() == -1)	{
			ROS_ERROR_STREAM("Motor initialization unsucessfull");
			return -1;
		}
		if(rc_encoders_init() == -1)	{
			ROS_ERROR_STREAM("Encoder initialization unsucessfull");
			return -1;
		}
	}




	while (ros::ok())
	{

		if (motor_state){
			double pow_1, pow_2, pow_3, pow_4;  //Based on v_x,v_y,a_z set power for each motor


			rc_motor_set(1, pow_1);
			rc_motor_set(2, pow_2);
			rc_motor_set(3, pow_3);
			rc_motor_set(4, pow_4);
			ROS_INFO(" %f ", v_x);
		}
		else {
			rc_motor_set(1,0);
			rc_motor_set(2,0);
			rc_motor_set(3,0);
			rc_motor_set(4,0);
			ROS_INFO("ne_dela");
		}
		pub.publish(spremenljivka);
		//ROS_INFO("Help me with cpp");


		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}