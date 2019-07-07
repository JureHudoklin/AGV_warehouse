#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <robotcontrol.h>

#include <dynamic_reconfigure/server.h>
#include <jure_test/ReconfigureConfig.h>

// export ROS_MASTER_URI=http://192.168.7.1:11311 on BBB 192.168.43.179
// export ROS_IP=192.168.7.2 on BBB
// rosrun tf static_transform_publisher 0 0 0 0 0 0 1 map my_frame 10

//GPIO setup
#define GP0_1 3, 17 //LED_S
#define GP0_2 3, 20 //DIG 2
#define GP0_3 1, 17 //DIG 1
#define GP0_4 1, 25 //DIG 0

#define GP1_1 2, 3
#define GP1_2 2, 2
#define GP1_3 3, 1
#define GP1_4 3, 2

int use_motors;
int use_line;
int sensorNum;
int ledState;
int sensorFB;

// Callbacks
void reconfigure_callback(jure_test::ReconfigureConfig &config, uint32_t level) {

	sensorNum = config.senNum;
	ledState = config.LED;
	sensorFB = config.senFB;

	return;
}





int main(int argc, char** argv) {
	ros::init(argc, argv, "jure_test_node");
	ros::NodeHandle nh;

	// Set node parameters
	nh.getParam("/jure_test/use_motors", use_motors);
	nh.getParam("/jure_test/use_line", use_line);


	// Set dynamic reconfigure
	dynamic_reconfigure::Server<jure_test::ReconfigureConfig> server;
	dynamic_reconfigure::Server<jure_test::ReconfigureConfig>::CallbackType rec_f;
	rec_f = boost::bind(&reconfigure_callback, _1, _2);
	server.setCallback(rec_f);



	
	
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

	if (use_line == 1) {
		ROS_INFO("Initializing line sensors");
		rc_adc_init();
		rc_gpio_init(GP0_1, GPIOHANDLE_REQUEST_OUTPUT); //LED_S
		rc_gpio_init(GP0_2, GPIOHANDLE_REQUEST_OUTPUT); //DIG_2
		rc_gpio_init(GP0_3, GPIOHANDLE_REQUEST_OUTPUT); //DIG_1
		rc_gpio_init(GP0_4, GPIOHANDLE_REQUEST_OUTPUT); //DIG_0
	}

	ros::Rate loop_rate(100);
	int pass = 0;
	ros::Time time_now;

	while (ros::ok())
	{
		ros::spinOnce();

		if (use_line) {

			if (ledState == 2) {
				double difference;

				int a, b, c;
				a = sensorNum & 0b001;
				b = sensorNum & 0b010;
				c = sensorNum & 0b100;
				rc_gpio_set_value(GP0_4, a);
				rc_gpio_set_value(GP0_3, b);
				rc_gpio_set_value(GP0_2, c);

				if (sensorFB != 2) {
					ros::Duration(0.01).sleep();

					rc_gpio_set_value(GP0_4, 0);
					ros::Duration(0.01).sleep();

					difference = rc_adc_read_volt(sensorFB+1);
					ros::Duration(0.1).sleep();

					rc_gpio_set_value(GP0_4, 1);
					ros::Duration(0.01).sleep();

					difference = difference - rc_adc_read_volt(sensorFB+1);
					ros::Duration(0.1).sleep();

					rc_gpio_set_value(GP0_4, 0);

					ROS_INFO("Sensor nm: %d, led: %d , value: %f" ,sensorNum, ledState, difference);
				}
				
			} else {
				rc_gpio_set_value(GP0_1, ledState);

				int a, b, c;
				a = sensorNum & 0b001;
				b = sensorNum & 0b010;
				c = sensorNum & 0b100;
				rc_gpio_set_value(GP0_4, a);
				rc_gpio_set_value(GP0_3, b);
				rc_gpio_set_value(GP0_2, c);

				ros::Duration(0.01).sleep();

				ROS_INFO("%d Sensor nm: %d, led: %d , value: %f" , sensorFB, sensorNum,ledState, rc_adc_read_volt(sensorFB));
				/*

				if (sensorFB == 0) {
					ROS_INFO("Front Sensor nm: %d, led: %d , value: %f" ,sensorNum,ledState,rc_adc_read_volt(1));
				}

				if (sensorFB == 1) {
					ROS_INFO("Front Sensor nm: %d, led: %d , value: %f" ,sensorNum,ledState,rc_adc_read_volt(4));
				}

				if (sensorFB == 2) {
					ROS_INFO("Front Sensor nm: %d, led: %d , value: %f" ,sensorNum,ledState,rc_adc_read_volt(1));
					ros::Duration(0.1).sleep();
					ROS_INFO("Front Sensor nm: %d, led: %d , value: %f" ,sensorNum,ledState,rc_adc_read_volt(4));
					
				}
				*/

			}
		}


		if (use_motors) {
			int m;
			int j;
			for (int i = 0; i<4; i++) {
				j = rc_encoder_read(i+1);
				ROS_INFO("Encoder %d, %d", i, j);
			}

			if(pass == 0) {
				rc_motor_set(1, 0.1);
				rc_motor_set(2, 0.1);
				rc_motor_set(3, 0.1);
				rc_motor_set(4, 0.1);
			}

			time_now = ros::Time::now();
			ROS_INFO("Cas 0: %f", time_now.toSec());
			//ros::Duration(10).sleep();
			int enc_val = 0;
			while(enc_val<6000) {
				enc_val = -rc_encoder_read(1);
			}
			time_now = ros::Time::now();
			ROS_INFO("Cas 0: %f", time_now.toSec());
			ROS_INFO("enc_val %d", enc_val);

			rc_motor_set(1,0);
			rc_motor_set(2,0);
			rc_motor_set(3,0);
			rc_motor_set(4,0);
			pass++;
			return 0;
		}

	
		loop_rate.sleep();
	}


	if (use_line) {
		rc_adc_cleanup();
        rc_gpio_cleanup(GP0_1);
        rc_gpio_cleanup(GP0_2);
        rc_gpio_cleanup(GP0_3);
        rc_gpio_cleanup(GP0_4);
	}

	if (use_motors) {
		rc_motor_cleanup();
		rc_encoder_cleanup();
	}

	return 0;
}
