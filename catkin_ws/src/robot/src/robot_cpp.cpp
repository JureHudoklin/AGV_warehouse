// http://mate.tue.nl/mate/pdfs/7566.pdf reference for robot controll
// http://www-ist.massey.ac.nz/conferences/ICARA2004/files/Papers/Paper74_ICARA2004_425_428.pdf

#include <ros/ros.h>
#include <string>
#include <vector>

#include <robotcontrol.h>

//Messages
#include <std_msgs/Int32.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Range.h>
#include <robot/Control.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <std_srvs/Empty.h>


//Structures
struct Robot_velocity_state {
    double v_x; //Front left
    double v_y; //Frpnt right
	double a_z; //Back right
};
struct Motor_power_state {
    double pow_1; //Front left
    double pow_2; //Frpnt right
	double pow_3; //Back right
	double pow_4; //Back left
};

//Publishers
ros::Publisher pub;

//Subscribers
ros::Subscriber cmd_vel_sub; 
           
//Services
ros::ServiceServer motors_on; 
ros::ServiceServer motors_off; 

//Define used variable
bool motor_state;
int use_motors, use_MPU;
Robot_velocity_state robot_velocity;

void vel2motor(Motor_power_state &pow) {
	double vmesni_izracun[4];
	double koti_koles[4] = {3.*M_PI/4., 5.*M_PI/4., 7.*M_PI/4., M_PI/4.};

	for(int i = 0; i<4; i++) {
		vmesni_izracun[i] = robot_velocity.a_z + robot_velocity.v_x * cos(koti_koles[i]) + robot_velocity.v_y * sin(koti_koles[i]);
	}

	//TUKAJ DOADAJ DA BO SKALIRALO OUTPUT DO 1
	pow.pow_1 = vmesni_izracun[1];
	pow.pow_2 = vmesni_izracun[2];
	pow.pow_3 = vmesni_izracun[3];
	pow.pow_4 = vmesni_izracun[4];
	return;
}

//Classes
class Robot {
	private:
		rc_mpu_data MPU_data;
	public:
		Robot(void);
		void dmp_callback(void);


};
Robot::Robot(void) {
	if (use_motors == 1)	{
		cmd_vel_sub = nh.subscribe("/cmd_vel", 100, &twist_callback); 
		if(rc_motor_init() == -1)	{
			ROS_ERROR_STREAM("Motor initialization unsucessfull");
			return -1;
		}
		if(rc_encoder_init() == -1)	{
			ROS_ERROR_STREAM("Encoder initialization unsucessfull");
		}
		for(int i = 0; i<4; i++) {
			if(rc_encoder_write(i, 0) == -1) {
				ROS_ERROR_STREAM("Encoder set 0 unsucessfull");
			}
		}
	}
	if (use_MPU == 1) {
		rc_mpu_config_t MPU_conf = rc_mpu_default_config();
		//MPU_conf.enable_magnetometer = 1;
		//MPU_conf.dmp_sample_rate = 50;
		//rc_mpu_orientation_t ORIENTATION_Z_UP;
		if(rc_mpu_initialize_dmp(&MPU_data, MPU_conf) == -1)	{
			ROS_ERROR_STREAM("MPU initialization unsucessfull");
			return -1;
		}
		rc_mpu_set_dmp_callback(dmp_callback);
	}
}
void Robot::dmp_callback(void) {
	ROS_INFO("podatki %f", MPU_data.accel[0]);
	return;
}


bool motors_on_call(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    motor_state = true;
    return true;
}

bool motors_off_call(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    motor_state = false;
    return true;
}

void twist_callback(const geometry_msgs::Twist &twist) {		//Zakaj kle const. Zakaj argument v fn ce ga spodi ne dam
	robot_velocity.v_x = twist.linear.x;
	robot_velocity.v_y = twist.linear.y;
	robot_velocity.a_z = twist.angular.z;
	ROS_INFO("nove_hitrosti");
	return;
}
//------------------------------------------------------------------------------------------------------------------//

int main(int argc, char** argv) {
    
    //Definer and run the node
	ros::init(argc, argv, "robot");
	ros::NodeHandle nh;

	//Set node parameters
	nh.getParam("/robot_node/use_motors", use_motors);
	nh.getParam("/robot_node/use_MPU", use_MPU);
	ROS_INFO(" %d", use_motors);

    //Start adverstisers, subscribers and services
    //pub = nh.advertise<std_msgs::Int32>("jure_topic", 1000);
    //sub = nh.subscribe("tema_subscriptiona", 1000, counterCallback); 
	

	motors_on = nh.advertiseService("/motors_on", motors_on_call);
	motors_off = nh.advertiseService("/motors_off", motors_off_call);


	ros::Rate loop_rate(100);



	//INITIALIZE MOTORS and ENCODERS
	/*
	Initializes the use of motors.
	Initializes the use of encoders.
	If initialization is unsucessfol returns error.
	*/
	
	




	while (ros::ok())
	{
		if (motor_state){
			Motor_power_state power;  //Based on v_x,v_y,a_z set power for each motor

			vel2motor(power);

			rc_motor_set(1, power.pow_1);
			rc_motor_set(2, power.pow_2);
			rc_motor_set(3, power.pow_3);
			rc_motor_set(4, power.pow_4);
		}
		else {
			rc_motor_set(1,0);
			rc_motor_set(2,0);
			rc_motor_set(3,0);
			rc_motor_set(4,0);
			ROS_INFO("ne_dela");
		}

		ros::spinOnce();
		loop_rate.sleep();
	}

	if (use_motors){
		rc_motor_cleanup();
	}
	if (use_MPU){
		rc_mpu_power_off();
	}
	return 0;
}