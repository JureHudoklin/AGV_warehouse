// http://mate.tue.nl/mate/pdfs/7566.pdf reference for robot controll
// http://www-ist.massey.ac.nz/conferences/ICARA2004/files/Papers/Paper74_ICARA2004_425_428.pdf
// ROS_MASTER_URI=http://192.168.7.1:11311 on BBB
// ROS_IP=192.168.7.2 on BBB



/*
Encoder: 12CPR
Motor: 50:1
*/
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

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <std_srvs/Empty.h>
#include <dynamic_reconfigure/server.h>
#include <robot/ReconfigureConfig.h>

#define ANGLE_PER_ENC_PULSE 0.01047197551
#define WHEEL_DIAMETER 38.1

//GPIO setup
#define GP0_4 3, 17
#define GP0_3 3, 20
#define GP0_2 1, 17
#define GP0_1 1, 25

#define GP1_4 2, 3
#define GP1_3 2, 2
#define GP1_2 3, 1
#define GP1_1 3, 2

//Structures
struct Robot_vel_state {
    double target_v[3]; 
	double actual_v[3]; 
};
struct Robot_vel_err {
	double prop_err[3];
	double integ_err[3];
	double diff_err[3];
};
struct Robot_enc_val {
	int enc[4];
	ros::Time timestamp;
};
struct PID_koef {
	double P;
	double I;
	double D;
};

//Publishers
ros::Publisher line_sen_pub; // topic = line_sen

//Subscribers
ros::Subscriber cmd_vel_sub; 
           
//Services
ros::ServiceServer motors_on; 
ros::ServiceServer motors_off; 

//Define used variable
bool motor_state;
int use_motors, use_MPU, use_TOF, use_line;

//--------------------------TEST_--------------
double power_on_wheel =0.;



//Classes
class Robot {
	private:
		Robot_vel_err robot_vel_err;
		PID_koef PID;
		double scaling_factor;
	public:
		//Initialize
		//Variables
		double line_values_f[8];
		double line_values_b[8];
		static rc_mpu_data_t MPU_data;
		//Structures
		Robot_vel_state robot_vel;
		Robot_enc_val robot_enc_val;
		//Methods
		void set_PID(double p, double i, double d);
		void set_scaling(double i);
		static void dmp_callback(void);
		void read_encoders(Robot_enc_val &old_val);
		void calc_velocities(Robot_enc_val &old_val);
		void vel2power(double (&pwr)[4]);
		void wheel_speed(double (&ws)[4] ,double velocities[3]);
};
void Robot::set_PID(double p, double i, double d) {
	PID.P = p;
	PID.I = i;
	PID.D = d;
	return;
}
void Robot::set_scaling(double i) {
	scaling_factor = i;
	return;
}
void Robot::dmp_callback(void) {

			return;
		}
void Robot::read_encoders(Robot_enc_val &old_val) {
	//Encoders numbered from front left -> counter cloackwise
	for (int i = 0; i<4; i++) {
		old_val.enc[i] = robot_enc_val.enc[i];
	}
	old_val.timestamp = robot_enc_val.timestamp;
	for (int i = 0; i<4; i++) {
		robot_enc_val.enc[i] = -rc_encoder_read(i+1);
	}
	robot_enc_val.timestamp = ros::Time::now();
	return;
}
void Robot::calc_velocities(Robot_enc_val &old_val) {
	robot_vel.actual_v[2] = MPU_data.gyro[2];	//Angular velocity is read directly from gyro

	//In the loop below velocity (mm/s) for each wheel is calculated
	double wheel_vel[4];
	for(int i = 0; i<4; i++) {
		int enc_diff = robot_enc_val.enc[i] - old_val.enc[i];
		double time_diff = robot_enc_val.timestamp.toSec() - old_val.timestamp.toSec();
		wheel_vel[i] = ((double)enc_diff * ANGLE_PER_ENC_PULSE) / time_diff;
		wheel_vel[i] = wheel_vel[i]*WHEEL_DIAMETER / 2.;
		ROS_INFO(" Actual wheel velocity: %f", wheel_vel[i]);
	}
	
	/*
	Robot velocities are calculated from wheel velocities. As the system ofeqations is overdefined,
	Two pairs of three wheels are used and  velocities from those calculations are averaged.
	*/
	double vx[2];
	double vy[2];
	vx[0] = (wheel_vel[0]-wheel_vel[1])/sqrt(2.);
	vy[0] = 0.5 * sqrt(2.)*(-wheel_vel[1]+wheel_vel[2]);

	vx[1] = 0.5 * sqrt(2.)*(-wheel_vel[2]+wheel_vel[3]);
	vy[1] = 0.5 * sqrt(2.)*(-wheel_vel[1]+wheel_vel[2]);

	// Calculated velocities are set for the robot object
	robot_vel.actual_v[0] = (vx[0]+vx[1]) / 2.;
	robot_vel.actual_v[1] = (vy[0]+vy[1]) / 2.;
	return;
}
void Robot::wheel_speed(double (&ws)[4], double velocities[3]) {
	for(int i = 0; i<4; i++) {
		double alpha = (5*M_PI/4.)+(double)i*M_PI/2.;
		double b = 100.;
		ws[i] = (b*velocities[2] - velocities[1]* cos(alpha) + velocities[0]*sin(alpha));
	}
	return;
}
void Robot::vel2power(double (&pwr)[4]) {
	//First new errors are calucalted
	double old_prop_err[3];
	for(int i = 0; i<3; i++) {
		old_prop_err[i] = robot_vel_err.prop_err[i];
	}

	double weighted_velocities[3];
	
	for(int i = 0; i<3; i++) {
		robot_vel_err.prop_err[i] = robot_vel.target_v[i] - robot_vel.actual_v[i];	//TEGA NE RABIM PROPORCIONALNO
		robot_vel_err.integ_err[i] = robot_vel_err.integ_err[i] + robot_vel_err.prop_err[i];
		robot_vel_err.diff_err[i] = old_prop_err[i] - robot_vel_err.prop_err[i];
		weighted_velocities[i] = PID.P * robot_vel.target_v[i] + PID.I * robot_vel_err.integ_err[i] + PID.D * robot_vel_err.diff_err[i];
		ROS_INFO("target v %f, actual %f",robot_vel.target_v[i], robot_vel.actual_v[i]);
	}

	double wheel_s[4];
	wheel_speed(wheel_s, weighted_velocities);

	for(int i = 0; i<4; i++) {
		pwr[i] = wheel_s[i]/scaling_factor;		//o.1 pwr -> 120mm/s -> 150mm/s x
		ROS_INFO("wheel speed: %f", wheel_s[i]);
	}
	return;
}
rc_mpu_data_t Robot::MPU_data;
Robot robot_OBJ;


//Callbacks
void reconfigure_callback(robot::ReconfigureConfig &config, uint32_t level) {
	robot_OBJ.set_PID(config.P_koef, config.I_koef, config.D_koef);
	robot_OBJ.set_scaling(config.sf);
	robot_OBJ.robot_vel.target_v[0] = config.x;
	robot_OBJ.robot_vel.target_v[1] = config.y;
	robot_OBJ.robot_vel.target_v[2] = config.z;
	power_on_wheel = config.x;

	ROS_INFO("%f, %f, %f", config.P_koef, config.I_koef, config.D_koef, config.x);
	return;
}

void twist_callback(const geometry_msgs::Twist &twist) {		//Zakaj kle const. Zakaj argument v fn ce ga spodi ne dam
	robot_OBJ.robot_vel.target_v[0] = twist.linear.x;
	robot_OBJ.robot_vel.target_v[1] = twist.linear.y;
	robot_OBJ.robot_vel.target_v[2] = twist.angular.z;
	ROS_INFO("nove_hitrosti");
	return;
}

//Servic callbacks
bool motors_on_call(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    motor_state = true;
    return true;
}

bool motors_off_call(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    motor_state = false;
    return true;
}


//------------------------------------------------------------------------------------------------------------------//


int main(int argc, char** argv) {
    
    //Define and run the node
	ros::init(argc, argv, "robot");
	ros::NodeHandle nh;

	//Set node parameters
	nh.getParam("/robot_node/use_motors", use_motors);
	nh.getParam("/robot_node/use_MPU", use_MPU);
	nh.getParam("/robot_node/use_line", use_line);
	nh.getParam("/robot_node/use_TOF", use_TOF);

	//Set dynamic reconfigure
	dynamic_reconfigure::Server<robot::ReconfigureConfig> server;
	dynamic_reconfigure::Server<robot::ReconfigureConfig>::CallbackType rec_f;
	rec_f = boost::bind(&reconfigure_callback, _1, _2);
	server.setCallback(rec_f);

    //Start adverstisers, subscribers and services
    //pub = nh.advertise<std_msgs::Int32>("jure_topic", 1000);
    //sub = nh.subscribe("tema_subscriptiona", 1000, counterCallback); 
	

	motors_on = nh.advertiseService("/motors_on", motors_on_call);
	motors_off = nh.advertiseService("/motors_off", motors_off_call);

	if (use_motors == 1)	{
		ROS_INFO("Initializing motors and encoders");
		cmd_vel_sub = nh.subscribe("/cmd_vel", 100, &twist_callback); 
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
			else {
				robot_OBJ.robot_enc_val.enc[i] = -rc_encoder_read(i);
			}	
		}
		robot_OBJ.robot_enc_val.timestamp = ros::Time::now();
	}
	if (use_MPU == 1) {
		ROS_INFO("Initializing MPU");
		rc_mpu_config_t MPU_conf = rc_mpu_default_config();
		MPU_conf.dmp_fetch_accel_gyro = 1;
		//MPU_conf.enable_magnetometer = 1;
		//MPU_conf.dmp_sample_rate = 50;
		//rc_mpu_orientation_t ORIENTATION_Z_UP;
		if(rc_mpu_initialize_dmp(&robot_OBJ.MPU_data, MPU_conf) == -1)	{
			ROS_ERROR_STREAM("MPU initialization unsucessfull");
		}
		rc_mpu_set_dmp_callback(Robot::dmp_callback);
	}
	if (use_line) {
		ROS_INFO("Initializing line senzors");
        rc_adc_init();
        rc_gpio_init(GP0_1, GPIOHANDLE_REQUEST_OUTPUT); //LED_S
        rc_gpio_init(GP0_2, GPIOHANDLE_REQUEST_OUTPUT); //DIG_0
        rc_gpio_init(GP0_3, GPIOHANDLE_REQUEST_OUTPUT); //DIG_1
        rc_gpio_init(GP0_4, GPIOHANDLE_REQUEST_OUTPUT); //DIG_2

        line_sen_pub = nh.advertise<robot::Line_sensor>("/line_sen", 100);
    }
	if (use_TOF) {

	}



	ros::Rate loop_rate(0.4);

	Robot_enc_val enc_val_old = {{0,0,0,0},ros::Time::now()};
	while (ros::ok())
	{
		if (use_motors) {
			double motor_power[4] = {0,0,0,0};	//Based on v_x,v_y,a_z set power for each motor

			if (motor_state){

				robot_OBJ.read_encoders(enc_val_old);
				robot_OBJ.calc_velocities(enc_val_old);
				robot_OBJ.vel2power(motor_power);
				  
				for(int i = 0; i<4; i++) {
					double m;
					m = motor_power[i];
					if(m > 0.4 || m<-0.4) {
						m = 0.;
						ROS_INFO("wheel %d, power EXCEED", i);
					}
					rc_motor_set(i+1, m);
				}
			}
			else {
				rc_motor_set(1,0);
				rc_motor_set(2,0);
				rc_motor_set(3,0);
				rc_motor_set(4,0);
			}
		}

        if (use_line) {
            robot::Line_sensor line_sen_msg;

            for(int j = 0; j<2; j++) {
                rc_gpio_set_value(GP0_4, j);

				

                for(int i = 0; i<8; i++) {
                    int a, b, c;
                    a = i & 0b001;
                    b = i & 0b010;
                    c = i & 0b100;
                    rc_gpio_set_value(GP0_1, a);
                    rc_gpio_set_value(GP0_2, b);
                    rc_gpio_set_value(GP0_3, c);

                    ros::Duration(0.05).sleep(); //Sleeps so that multiplexer has time to settle
					//ROS_INFO(" %f, %d back", rc_adc_read_volt(4), j);
					
					//ROS_INFO(" %f, %d front", rc_adc_read_volt(3), j);
					//ros::Duration(0.5).sleep();

                    if (j == 0) {
                        robot_OBJ.line_values_f[i] = rc_adc_read_volt(3);
                        robot_OBJ.line_values_b[i] = rc_adc_read_volt(4);
						//ROS_INFO(" %f, %d back", rc_adc_read_volt(4), j);
                    }
                    else {
                        robot_OBJ.line_values_f[i] = -rc_adc_read_volt(3)+robot_OBJ.line_values_f[i];
                        robot_OBJ.line_values_b[i] = -rc_adc_read_volt(4)+robot_OBJ.line_values_b[i];
						//ROS_INFO(" %f, %d back", rc_adc_read_volt(4), j);
                    }
                }
            }
            rc_gpio_set_value(GP0_4, 0);
			//ROS_INFO(" %f", robot_OBJ.line_values_f[0]);
			//ROS_INFO(" %f", robot_OBJ.line_values_b[0]);

			for (int i = 0; i<8; i++) {
				line_sen_msg.front_sensors[i] = robot_OBJ.line_values_f[i];
				line_sen_msg.back_sensors[i] = robot_OBJ.line_values_b[i];
			}
			line_sen_pub.publish(line_sen_msg);			
        }

		if (use_TOF) {

		}

		if (use_MPU) {
			//robot_OBJ.calc_vel_err();
			//rc_mpu_read_gyro(&robot_OBJ.MPU_data);
			//ROS_INFO(" %f,%f,%f", robot_OBJ.MPU_data.gyro[2], robot_OBJ.MPU_data.gyro[1], robot_OBJ.MPU_data.gyro[0]);
			//ROS_INFO(" %f", robot_OBJ.MPU_data.accel[2], robot_OBJ.MPU_data.accel[1]);
			//ROS_INFO(" %f","%f", robot_OBJ.MPU_data.dmp_quat[0], robot_OBJ.MPU_data.dmp_quat[1]);
		}	

		ros::spinOnce();
		loop_rate.sleep();
	}


	ROS_INFO("Running device cleanup");
	if (use_motors){
		rc_motor_cleanup();
		rc_encoder_cleanup();
	}
	if (use_MPU){
		rc_mpu_power_off();
	}
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