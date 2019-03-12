#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <robotcontrol.h>

//GPIO setup
#define GP0_1 3, 17
#define GP0_2 3, 20
#define GP0_3 1, 17
#define GP0_4 1, 25

#define GP1_1 2, 3
#define GP1_2 2, 2
#define GP1_3 3, 1
#define GP1_4 3, 2

int main(int argc, char** argv) {
	ros::init(argc, argv, "jure_test_node");
	ros::NodeHandle nh;

	ros::Publisher pub = nh.advertise<std_msgs::Int32>("jure_topic", 1000);
	
	std_msgs::Int32 spremenljivka;
	spremenljivka.data = 0;
	rc_adc_init();
	rc_gpio_init(GP0_1, GPIOHANDLE_REQUEST_OUTPUT); //LED_S
	rc_gpio_init(GP0_2, GPIOHANDLE_REQUEST_OUTPUT); //DIG_0
	rc_gpio_init(GP0_3, GPIOHANDLE_REQUEST_OUTPUT); //DIG_1
	rc_gpio_init(GP0_4, GPIOHANDLE_REQUEST_OUTPUT); //DIG_2
	
	int use_motors = 1;
	if (use_motors == 1)	{
		ROS_INFO("Initializing motors and encoders");
		//cmd_vel_sub = nh.subscribe("/cmd_vel", 100, &twist_callback); 
		if(rc_motor_init() == -1)	{
			ROS_ERROR_STREAM("Motor initialization unsucessfull");
		}
		if(rc_encoder_init() == -1)	{
			ROS_ERROR_STREAM("Encoder initialization unsucessfull");
		}
		for(int i = 1; i<5; i++) {
			if(rc_encoder_write(i, 0) == -1) {
				ROS_ERROR_STREAM("Encoder set 0 unsucessfull");
			}	
		}
	}


	ros::Rate loop_rate(0.5);
	int pass = 0;

	while (ros::ok())
	{
		pub.publish(spremenljivka);
		ROS_INFO("Help me with cpp");
		ros::spinOnce();
		++spremenljivka.data;

		int m;
		int j;
		for (int i = 0; i<4; i++) {
			j = rc_encoder_read(i+1);
			ROS_INFO("Encoder %d, %d", i, j);
		}

		if(pass == 0) {
			rc_motor_set(1, 0.2);
			ROS_INFO("motor on");
			rc_motor_set(2, 0.2);
			rc_motor_set(3, 0.2);
			rc_motor_set(4, 0.2);
		}
		ros::Duration(1).sleep();
		for (int i = 0; i<4; i++) {
			j = rc_encoder_read(i+1);
			ROS_INFO("Encoder after %d, %d", i, j);
		}
		rc_motor_set(1,0);
		rc_motor_set(2,0);
		rc_motor_set(3,0);
		rc_motor_set(4,0);
		pass++;





		loop_rate.sleep();

		
	}
	rc_motor_cleanup();
	rc_encoder_cleanup();
	rc_adc_cleanup();
	return 0;
}
