#include <math.h>
#include <ros/ros.h>
#include <string>
#include <vector>

#include <robotcontrol.h>
#include "vl53l0x.h"
#include "robot.h"

//Messages
#include <std_msgs/Int32.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Range.h>
#include <robot/Control.h>
#include <robot/Line_sensor.h>
#include <robot/coordinate_sys_rotate.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include <std_srvs/Empty.h>
#include <dynamic_reconfigure/server.h>
#include <robot/ReconfigureConfig.h>

#define DEGREE2RADIANS 0.01745329252
#define DISTANCE_BETWEEN_SENSORS 9


//GPIO setup
#define GP0_1 3, 17 //LED_S
#define GP0_2 3, 20 //DIG 2
#define GP0_3 1, 17 //DIG 1
#define GP0_4 1, 25 //DIG 0

#define GP1_4 2, 3
#define GP1_3 2, 2
#define GP1_2 3, 1
#define GP1_1 3, 2

float* calculateLinePosition(double* front_sen, double* back_sen) {
        float* line_pos = new float[2];

        int f = getMinIndex<double>(front_sen+1, 6)+1;
        int b = getMinIndex<double>(back_sen+1, 6)+1;

        double f_v, b_v;

        f_v = -(f-3.5) - 0.33*(front_sen[f-1] - front_sen[f+1]);    
        b_v = -(b-3.5) - 0.33*(back_sen[b-1] - back_sen[b+1]);

        line_pos[0] = DISTANCE_BETWEEN_SENSORS * (float)f_v;

        line_pos[1] = DISTANCE_BETWEEN_SENSORS * (float)b_v;

        return line_pos;
}

//Publishers
ros::Publisher line_sen_pub; // topic = line_sen
ros::Publisher line_sen_pos_pub;
ros::Publisher tof_pub;

int use_line;
int use_TOF;

VL53L0X vl53l0x;

int main(int argc, char** argv) {


    //Definer and run the node
    ros::init(argc, argv, "robot_sensors_node");
	ros::NodeHandle nh;
    tf::TransformBroadcaster tof_broadcaster;

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
        line_sen_pos_pub = nh.advertise<geometry_msgs::PolygonStamped>("/line_pos", 1);
    }
    // ToF init
    if (use_TOF)
    {
        ROS_INFO_STREAM("Initializing ToF...");
        tof_pub = nh.advertise<sensor_msgs::Range>("tof/range", 10);
        if (vl53l0x.init())
        {
            ROS_INFO_STREAM("ToF initialized");
            vl53l0x.startContinuous();
        }
        else
        {
            ROS_ERROR_STREAM("failed to initialize TOF");
            return -1;
        }
    }
       

    //Set time variables and ros loop rate
    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    ros::Rate loop_rate(30);

    while (ros::ok())
	{
        last_time = current_time;
        current_time = ros::Time::now();
        
        if (use_line) {
            double line_values_f[8];
            double line_values_b[8];
            robot::Line_sensor line_sen_msg;
            geometry_msgs::PolygonStamped line;


            for(int j = 0; j<2; j++) {
                rc_gpio_set_value(GP0_1, j); //LED_S
                ros::Duration(0.001).sleep();


                for(int i = 0; i<8; i++) {
                    int a, b, c;
                    a = i & 0b001;
                    b = i & 0b010;
                    c = i & 0b100;
                    rc_gpio_set_value(GP0_4, a);    //DIG_0
                    rc_gpio_set_value(GP0_3, b);    //DIG_1
                    rc_gpio_set_value(GP0_2, c);    //DIG_2

                    ros::Duration(0.001).sleep(); //Sleeps so that multiplexer has time to settle

                    if (j == 0) {
                        line_values_f[i] = rc_adc_read_volt(1);
                        //ros::Duration(0.005).sleep();
                        line_values_b[i] = rc_adc_read_volt(3);
                    }
                    else {
                        line_values_f[i] = -rc_adc_read_volt(1)+line_values_f[i];
                        //ros::Duration(0.005).sleep();
                        line_values_b[i] = -rc_adc_read_volt(3)+line_values_b[i];
                    }
                }
            }
            rc_gpio_set_value(GP0_4, 0);
            double sum_f, sum_b; // Use this to determine if no line is detected or all sensors are on line

			for (int i = 0; i<8; i++) {
				line_sen_msg.front_sensors[i] = line_values_f[i];
				line_sen_msg.back_sensors[i] = line_values_b[i];
                sum_f += line_values_f[i];
                sum_b += line_values_b[i];
			}
            
            float* line_position;

            /* Add here an if statemen to see if no line is detected or all sensors on line */


            line_position = calculateLinePosition(line_values_f, line_values_b); // Line Position is calculated

            geometry_msgs::Point32 p_f;
            geometry_msgs::Point32 p_f_2; // This point only to show in RViz
            geometry_msgs::Point32 p_b;
            geometry_msgs::Point32 p_b_2; // This point only to show in RViz


            p_f.x = 0.2;
            p_f.y = line_position[0]/1000;
            p_f.z = 0;

            p_f_2.x = 0.2;
            p_f_2.y = line_position[0]/1000 +0.03;
            p_f_2.z = 0;

            p_b.x = -0.2;
            p_b.y = line_position[1]/1000;
            p_b.z = 0;

            p_b_2.x = -0.2;
            p_b_2.y = line_position[1]/1000 +0.03;
            p_b_2.z = 0;

            line.polygon.points.push_back(p_f);
            line.polygon.points.push_back(p_f_2);   // RViz
            line.polygon.points.push_back(p_b_2);   // RViz
            line.polygon.points.push_back(p_b);

            line_sen_msg.time_stamp = current_time;
            line.header.stamp = current_time;
            line.header.frame_id = "robot";

			line_sen_pub.publish(line_sen_msg);	
            line_sen_pos_pub.publish(line);	
            

            delete[] line_position;	
        }

        if (use_TOF)
        {


            //Pupulate tof_trans
			geometry_msgs::TransformStamped tof_trans;
			tof_trans.header.stamp = current_time;
			tof_trans.header.frame_id = "robot";
			tof_trans.child_frame_id = "tof";

			tof_trans.transform.translation.x = 0.07;
			tof_trans.transform.translation.y = 0;
			tof_trans.transform.translation.z = 0.02;
			tof_trans.transform.rotation.x = 0;
            tof_trans.transform.rotation.y = 0;
            tof_trans.transform.rotation.z = 0;
            tof_trans.transform.rotation.w = 1;

			//Brodcast transform
			tof_broadcaster.sendTransform(tof_trans);

            
            // ToF
            sensor_msgs::Range tof_msg;
            tof_msg.header.stamp = current_time;
            tof_msg.header.frame_id = "robot";
            tof_msg.field_of_view = 25.0 * M_PI / 180.0; // 25 degrees acc. to datasheet
            tof_msg.min_range = 0.0;
            tof_msg.max_range = 1.0;
            tof_msg.range = vl53l0x.readRangeContinuousMillimeters() / 1000.0;
            tof_pub.publish(tof_msg);
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
    if (use_TOF)
    {
        vl53l0x.stopContinuous();
    }
    
	return 0;
}
