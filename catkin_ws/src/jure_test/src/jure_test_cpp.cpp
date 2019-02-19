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
	ros::Rate loop_rate(20);

	while (ros::ok())
	{
		pub.publish(spremenljivka);
		ROS_INFO("Help me with cpp");
		ros::spinOnce();
		++spremenljivka.data;

		int m;

		for (int i = 0;i<8;i++) {
			for (int k = 0;k<3;k++) {
				m = (i & ( 1 << k )) >> k;
			}
		rc_gpio_set_value(GP0_2, 0);
		ros::Duration(2).sleep();
		rc_gpio_set_value(GP0_2, 1);
		ros::Duration(2).sleep();	

		}

		loop_rate.sleep();

		
	}
	rc_adc_cleanup();
	return 0;
}
