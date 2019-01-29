
#include <math.h>
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

#define GP0_1 3, 17
#define GP0_2 3, 20
#define GP0_3 1, 17
#define GP0_4 1, 25

#define GP1_1 2, 3
#define GP1_2 2, 2
#define GP1_3 3, 1
#define GP1_4 3, 2


int line_values_f[8];
int line_values_b[8];
int use_line = 1;

struct line_error {
    double vel_y;
    double vel_z;
};

void calc_line_error(line_error &arr, int* front_s, int* back_s) {
    /*THis function calculates velocities in y direction and rotational vel z
    based on readings of 4 cetral sensors on front and back. Velocity values 
    is between -1 and 1. Sensor weight is set with parameter a.*/
    double front[2];
    double back[2];
    long all_frt[4];
    long all_back[4];
    double a;
    a = 0.7;

    for(int i = 2; i<6; i++) {
        all_frt[i-2] = pow(front_s[i], 2);
        all_back[i-2] = pow(back_s[i], 2);
    }

    front[0] = a * (double)all_frt[1] + (1.- a) * (double)all_frt[0];
    front[1] = a * (double)all_frt[2] + (1.- a) * (double)all_frt[3];
    back[0] = a * (double)all_back[1] + (1.- a) * (double)all_back[0];
    back[1] = a * (double)all_back[2] + (1.- a) * (double)all_back[3];

    arr.vel_y = (front[1]-front[0]+back[1]-back[0])/(2.*pow(2.,16.));
    arr.vel_z = (front[1]-front[0]-back[1]+back[0])/(2.*pow(2.,16.));
    return;
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "robot_control");
    ros::NodeHandle nh;

    nh.getParam("/robot_control_node/use_line", use_line);
    ros::Rate loop_rate(50);
    


    //Initialze giop and adc for line sensors
    if (use_line) {

        rc_adc_init();
        rc_gpio_init(GP0_1, GPIOHANDLE_REQUEST_OUTPUT); //LED_S
        rc_gpio_init(GP0_2, GPIOHANDLE_REQUEST_OUTPUT); //DIG_0
        rc_gpio_init(GP0_3, GPIOHANDLE_REQUEST_OUTPUT); //DIG_1
        rc_gpio_init(GP0_4, GPIOHANDLE_REQUEST_OUTPUT); //DIG_2
    }


    while(ros::ok()) {
        /* I run everything needed for line sensors. I cycle trugh all senors on and off and store
        the difference between two in line_values frond and back
        --------------------------------------------------------
        */
        line_error err_vel;

        if (use_line) {
            for(int j = 0; j<2; j++) {
                rc_gpio_set_value(GP0_1, j);

                for(int i = 0; i<8; i++) {
                    int a, b, c;
                    a = i & 0b001;
                    b = i & 0b010;
                    c = i & 0b100;
                    rc_gpio_set_value(GP0_2, a);
                    rc_gpio_set_value(GP0_3, b);
                    rc_gpio_set_value(GP0_4, c);

                    ros::Duration(0.00001).sleep(); //Slleps so that multiplexer has ime to settle

                    if (j == 0) {
                        line_values_f[i] = rc_adc_read_raw(0);
                        line_values_b[i] = rc_adc_read_raw(1);
                    }
                    else {
                        line_values_f[i] = rc_adc_read_raw(0)-line_values_f[i];
                        line_values_b[i] = rc_adc_read_raw(1)-line_values_b[i];
                    }
                }
            }
            rc_gpio_set_value(GP0_1, 0);
            calc_line_error(err_vel, line_values_f, line_values_b);
            ROS_INFO(" %f ", err_vel.vel_y);
        }
        /*---------------------------------------------------------*/
        ros::spinOnce();
	    loop_rate.sleep();
        
    }


    /*Close all used devices on the end*/
    if (use_line) {
        rc_adc_cleanup();
        rc_gpio_cleanup(GP0_1);
        rc_gpio_cleanup(GP0_2);
        rc_gpio_cleanup(GP0_3);
        rc_gpio_cleanup(GP0_4);
    }
    
    return 0;
}

