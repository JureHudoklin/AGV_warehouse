#include <math.h>
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
#include <robot/Line_sensor.h>
#include <robot/coordinate_sys_rotate.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include <std_srvs/Empty.h>
#include <dynamic_reconfigure/server.h>
#include <robot/ReconfigureConfig.h>

#define ANGLE_PER_ENC_PULSE 0.01047197551
#define WHEEL_DIAMETER 38.1
#define DEGREE2RADIANS 0.01745329252

//GPIO setup
#define GP0_1 3, 17 //LED_S
#define GP0_2 3, 20 //DIG 2
#define GP0_3 1, 17 //DIG 1
#define GP0_4 1, 25 //DIG 0

#define GP1_4 2, 3
#define GP1_3 2, 2
#define GP1_2 3, 1
#define GP1_1 3, 2

//Publishers
ros::Publisher line_sen_pub; // topic = line_sen
int use_line;
int use_TOF;

int main(int argc, char** argv) {

    //Definer and run the node
    ros::init(argc, argv, "robot_sensors_node");
	ros::NodeHandle nh;

    //Set node parameters
    nh.getParam("/robot_sensors_node/use_line", use_line);
    nh.getParam("/robot_sensors_node/use_TOF", use_TOF);

    ROS_INFO("Starting robot_sensors");

    if (use_line) {
		ROS_INFO("Initializing line senzors");
        rc_adc_init();
        rc_gpio_init(GP0_1, GPIOHANDLE_REQUEST_OUTPUT); //LED_S
        rc_gpio_init(GP0_2, GPIOHANDLE_REQUEST_OUTPUT); //DIG_2
        rc_gpio_init(GP0_3, GPIOHANDLE_REQUEST_OUTPUT); //DIG_1
        rc_gpio_init(GP0_4, GPIOHANDLE_REQUEST_OUTPUT); //DIG_0

        line_sen_pub = nh.advertise<robot::Line_sensor>("/line_sen", 1);
    }
    if (use_TOF) {
        ROS_INFO("Initializing TOF sensor");
    }

    //Set time variables and ros loop rate
    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    ros::Rate loop_rate(10);

    while (ros::ok())
	{
        last_time = current_time;
        current_time = ros::Time::now();

        /*   IZBRISI POTEM KO BO DELOVALO BRANJE CRTE ------------------------------------!!!!!!
        while(1) {
            rc_gpio_set_value(GP0_4, 1);
            ros::Duration(2).sleep();
            rc_gpio_set_value(GP0_4, 0);
            ros::Duration(2).sleep();
        }
        
        while(1) {
            for(int i = 0; i<8; i++) {
                    int a, b, c;
                    a = i & 0b001;
                    b = (i & 0b010)>>1;
                    c = (i & 0b100)>>1;
                    ROS_INFO(" a = %d, b = %d, c = %d ", a,b,c);
                    rc_gpio_set_value(GP0_1, a);
                    rc_gpio_set_value(GP0_2, b);
                    rc_gpio_set_value(GP0_3, c);
                    rc_gpio_set_value(GP0_4, a);
                    ros::Duration(4).sleep();
                    ROS_INFO(" %f", rc_adc_read_volt(4));
                    ROS_INFO(" %f", rc_adc_read_volt(3));
            }
        }
        */
        
        
        if (use_line) {
            double line_values_f[8];
            double line_values_b[8];
            robot::Line_sensor line_sen_msg;

            for(int j = 0; j<2; j++) {
                rc_gpio_set_value(GP0_1, j); //LED_S
                ros::Duration(0.02).sleep();


                for(int i = 0; i<8; i++) {
                    int a, b, c;
                    a = i & 0b001;
                    b = i & 0b010;
                    c = i & 0b100;
                    rc_gpio_set_value(GP0_4, a);    //DIG_0
                    rc_gpio_set_value(GP0_3, b);    //DIG_1
                    rc_gpio_set_value(GP0_2, c);    //DIG_2

                    ros::Duration(0.005).sleep(); //Sleeps so that multiplexer has time to settle

                    if (j == 0) {
                        line_values_f[i] = rc_adc_read_volt(1);
                        ros::Duration(0.005).sleep();
                        line_values_b[i] = rc_adc_read_volt(3);
                    }
                    else {
                        line_values_f[i] = -rc_adc_read_volt(1)+line_values_f[i];
                        ros::Duration(0.005).sleep();
                        line_values_b[i] = -rc_adc_read_volt(3)+line_values_b[i];
                    }
                }
            }
            rc_gpio_set_value(GP0_4, 0);

			for (int i = 0; i<8; i++) {
				line_sen_msg.front_sensors[i] = line_values_f[i];
				line_sen_msg.back_sensors[i] = line_values_b[i];
			}
			line_sen_pub.publish(line_sen_msg);			
        }

        if (use_TOF) {

	    }

		ros::spinOnce();    //On spin it checks messages
		loop_rate.sleep();
	}



    /* CLEANUP */
    if (use_line) {
		rc_adc_cleanup();
        rc_gpio_cleanup(GP0_1);
        rc_gpio_cleanup(GP0_2);
        rc_gpio_cleanup(GP0_3);
        rc_gpio_cleanup(GP0_4);
	}
    if (use_TOF) {

	}
	return 0;
}
