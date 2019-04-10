// http://mate.tue.nl/mate/pdfs/7566.pdf reference for robot controll
// http://www-ist.massey.ac.nz/conferences/ICARA2004/files/Papers/Paper74_ICARA2004_425_428.pdf
// export ROS_MASTER_URI=http://192.168.43.179:11311 on BBB 192.168.43.179
// export ROS_IP=192.168.43.149 on BBB
// scp -r ubuntu@192.168.7.2:~/catkin_ws/src/ /home/jure/jure_ROS/AGV_warehouse/catkin_ws/

// IMU filters
// extended calman filter
// https://books.google.si/books?id=68RiDwAAQBAJ&pg=PA90&lpg=PA90&dq=ros+set+action+goal+during+execution&source=bl&ots=DyUcfHj9j-&sig=ACfU3U3VQUSoKjwrw1PxcrYYjk_HlbGJ0w&hl=sl&sa=X&ved=2ahUKEwj2jdnk6r3hAhVnxKYKHUKeDMAQ6AEwA3oECAgQAQ#v=onepage&q=ros%20set%20action%20goal%20during%20execution&f=false



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
#include <robot/coordinate_sys_rotate.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

// Other
#include <std_srvs/Empty.h>
#include <dynamic_reconfigure/server.h>
#include <robot/ReconfigureConfig.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <robot/MoveRobotAction.h>
#include <boost/thread.hpp>

#define ANGLE_PER_ENC_PULSE 0.01047197551
#define WHEEL_DIAMETER 38.1
#define DEGREE2RADIANS 0.01745329252

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
    double target_v[3]; //Is set based on the Koordinate system used (global or robot)
	double actual_v[3]; //Is always the velocity in robot KS
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
struct Vel_Pose {
	double x,y,z;
	double a_x, a_y, a_z;
};


//Publishers
ros::Publisher odom_pub;

//Subscribers
ros::Subscriber cmd_vel_sub; 
           
//Services
ros::ServiceServer motors_on; 
ros::ServiceServer motors_off;
ros::ServiceServer KS_rotate; 



//Define used variable
bool motor_state;
int use_motors, use_MPU, use_TOF;

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
		static double coordinate_ofset;
		ros::Time current_time, last_time;
		static rc_mpu_data_t MPU_data;
		//Structures
		Robot_vel_state robot_vel;
		Robot_enc_val robot_enc_val;
		Vel_Pose vel_pose;
		//Methods
		void set_PID(double p, double i, double d);
		void set_scaling(double i);
		static void dmp_callback(void);
		void read_encoders(Robot_enc_val &old_val);
		void calc_velocities(Robot_enc_val &old_val);
		void weigh_velocities(double(& weighted_velocities)[3], double pr_err[3]);
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
	//Add here if any callback is needed after new dmp_data
	return;
}
void Robot::read_encoders(Robot_enc_val &old_val) {
	//Encoders numbered from front left -> counter cloackwise
	for (int i = 0; i<4; i++) {
		old_val.enc[i] = robot_enc_val.enc[i];
	}
	old_val.timestamp = robot_enc_val.timestamp;
	for (int i = 0; i<4; i++) {
		robot_enc_val.enc[i] = rc_encoder_read(i+1);
	}
	robot_enc_val.timestamp = ros::Time::now();
	return;
}
void Robot::calc_velocities(Robot_enc_val &old_val) {
	robot_vel.actual_v[2] = round(MPU_data.gyro[2]) * DEGREE2RADIANS;	//Angular velocity is read directly from gyro

	//In the loop below velocity (mm/s) for each wheel is calculated
	double wheel_vel[4];
	for(int i = 0; i<4; i++) {
		int enc_diff = robot_enc_val.enc[i] - old_val.enc[i];
		double time_diff = robot_enc_val.timestamp.toSec() - old_val.timestamp.toSec();
		wheel_vel[i] = ((double)enc_diff * ANGLE_PER_ENC_PULSE) / time_diff;
		wheel_vel[i] = wheel_vel[i]*WHEEL_DIAMETER / 2.;
		//ROS_INFO(" Actual wheel velocity: %f", wheel_vel[i]);
	}
	
	/*
	Robot velocities are calculated from wheel velocities. As the system ofeqations is overdefined,
	Two pairs of three wheels are used and  velocities from those calculations are averaged.
	*/
	double vx[3];
	double vy[3];
	vx[0] = wheel_vel[2]*sqrt(2.)/2.-wheel_vel[1]*sqrt(2.)/2.;   
	vy[0] = -wheel_vel[1]*sqrt(2.)/2.+wheel_vel[0]*sqrt(2.)/2.;   

	vx[1] = 0.5 * sqrt(2.)*(-wheel_vel[1]+wheel_vel[2]);
	vy[1] = 0.5 * sqrt(2.)*(-wheel_vel[2]+wheel_vel[3]);

	vx[2] = 0.5 * sqrt(2.)*(-wheel_vel[0]+wheel_vel[3]);
	vy[2] = 0.5 * sqrt(2.)*(-wheel_vel[2]+wheel_vel[3]);

	// Calculated velocities are set for the robot object
	robot_vel.actual_v[0] = (vx[0]+vx[1]+vx[2]) / 3.; 
	robot_vel.actual_v[1] = (vy[0]+vy[1]+vy[2]) / 3.; 
	return;
}
void Robot::wheel_speed(double (&ws)[4], double velocities[3]) {
	for(int i = 0; i<4; i++) {
		double alpha = (3*M_PI/4.)+(double)i*M_PI/2.;
		double b = 100.;
		ws[i] = (b*velocities[2] + velocities[1]* sin(alpha) + velocities[0]*cos(alpha));
		ws[i] += 100. * abs(ws[i])/ws[i];
	}
	return;
}
void Robot::weigh_velocities(double(& weighted_velocities)[3], double pr_err[3]) {
	double dt = current_time.toSec()-last_time.toSec();

	for(int i = 0; i<3; i++) {

		robot_vel_err.prop_err[i] = robot_vel.target_v[i] - robot_vel.actual_v[i];
		robot_vel_err.integ_err[i] = robot_vel_err.integ_err[i] + robot_vel_err.prop_err[i]*dt;
		robot_vel_err.diff_err[i] = pr_err[i] - robot_vel_err.prop_err[i];
	

		weighted_velocities[i] = PID.P * robot_vel_err.prop_err[i] + PID.I * robot_vel_err.integ_err[i] + PID.D * robot_vel_err.diff_err[i];
		ROS_INFO("target v %f, actual %f",robot_vel.target_v[i], robot_vel.actual_v[i]);
	}

	return;
}
void Robot::vel2power(double (&pwr)[4]) {
	//First new errors are calucalted
	double old_prop_err[3];
	int integral_on = 0;
	for(int i = 0; i<3; i++) {
		if(robot_vel.target_v[i] != 0) {
			integral_on = 1;
		}
		old_prop_err[i] = robot_vel_err.prop_err[i];
	}
	
	double weighted_vel[3] = {0,0,0};
	double wheel_s[4] = {0,0,0,0};
	if (integral_on == 1) {
		weigh_velocities(weighted_vel, old_prop_err);
		wheel_speed(wheel_s, weighted_vel);
	}
	else {
		for(int i = 0; i<3; i++) {
			robot_vel_err.integ_err[i]= 0;
		}
	}

	for(int i = 0; i<4; i++) {
		pwr[i] = wheel_s[i]/scaling_factor;		//o.1 pwr -> 120mm/s -> 150mm/s x
		//ROS_INFO("wheel speed: %f", wheel_s[i]);
	}
	return;
}
rc_mpu_data_t Robot::MPU_data;
double Robot::coordinate_ofset = 0;
Robot robot_OBJ;


// Callbacks
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

void twist_callback(const geometry_msgs::Twist &twist) {
	
	robot_OBJ.robot_vel.target_v[0] = twist.linear.x;
	robot_OBJ.robot_vel.target_v[1] = twist.linear.y;
	robot_OBJ.robot_vel.target_v[2] = twist.angular.z;
	ROS_INFO("nove_hitrosti");
	return;
}

// Action callbacks
void activeCB() {
	// Called once when goal becomes active
	ROS_INFO("Moving robot to new location");
}
void feedbackCB(const robot::MoveRobotFeedbackConstPtr& feedback) {
	for (int i = 0; i<3; i++) {
		robot_OBJ.robot_vel.target_v[i]= feedback->velocity[i];
	}
}
void doneCB(const actionlib::SimpleClientGoalState& state,
			const robot::MoveRobotResultConstPtr& result) {
	
	ROS_INFO("Finished move");
}


// Servic callbacks
bool motors_on_call(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    motor_state = true;
    return true;
}

bool motors_off_call(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    motor_state = false;
    return true;
}
bool KS_rotate_call(robot::coordinate_sys_rotate::Request &req,
					robot::coordinate_sys_rotate::Response &res) {

	double ch_zero = -robot_OBJ.vel_pose.a_z;
	robot_OBJ.coordinate_ofset = ch_zero + req.KS_ofset;
	res.success = 1;

	return true;
}

// Functions

//------------------------------------------------------------------------------------------------------------------//


int main(int argc, char** argv) {
    
    // Define and run the node
	ros::init(argc, argv, "robot");
	ros::NodeHandle nh;

	tf::TransformBroadcaster odom_broadcaster;

	

	// Set node parameters
	nh.getParam("/robot_node/use_motors", use_motors);
	nh.getParam("/robot_node/use_MPU", use_MPU);
	nh.getParam("/robot_node/use_TOF", use_TOF);

	// Set dynamic reconfigure
	dynamic_reconfigure::Server<robot::ReconfigureConfig> server;
	dynamic_reconfigure::Server<robot::ReconfigureConfig>::CallbackType rec_f;
	rec_f = boost::bind(&reconfigure_callback, _1, _2);
	server.setCallback(rec_f);

    //Start adverstisers, subscribers and services
    //pub = nh.advertise<std_msgs::Int32>("jure_topic", 1000);
    //sub = nh.subscribe("tema_subscriptiona", 1000, counterCallback); 
	

	motors_on = nh.advertiseService("/motors_on", motors_on_call);
	motors_off = nh.advertiseService("/motors_off", motors_off_call);
	KS_rotate = nh.advertiseService("/KS_rotate", KS_rotate_call);

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
				robot_OBJ.robot_enc_val.enc[i] = rc_encoder_read(i);
			}	
		}
		robot_OBJ.robot_enc_val.timestamp = ros::Time::now();
	}
	if (use_MPU == 1) {
		ROS_INFO("Initializing MPU");
		rc_mpu_config_t MPU_conf = rc_mpu_default_config();
		MPU_conf.dmp_fetch_accel_gyro = 1;
		MPU_conf.enable_magnetometer = 1;
		MPU_conf.compass_time_constant = 25.;
		MPU_conf.dmp_sample_rate = 200;
		if(rc_mpu_initialize_dmp(&robot_OBJ.MPU_data, MPU_conf) == -1)	{
			ROS_ERROR_STREAM("MPU initialization unsucessfull");
		}
		robot_OBJ.vel_pose.x = 0;
		robot_OBJ.vel_pose.y = 0;
		robot_OBJ.vel_pose.z = 0;
		robot_OBJ.vel_pose.a_x = 0;
		robot_OBJ.vel_pose.a_y = 0;
		robot_OBJ.vel_pose.a_z = 0;


		odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
		rc_mpu_set_dmp_callback(Robot::dmp_callback);
	}

	// Create action client
	
	actionlib::SimpleActionClient<robot::MoveRobotAction> ac("robot_MoveRobot_node");
	ROS_INFO("Waiting for action server to start.");
	ac.waitForServer();
	ROS_INFO("Action server started");
	robot::MoveRobotGoal goal;
	goal.speed = 200.;
	goal.target[0] = 100.;
	goal.target[1] = 300.;
	ac.sendGoal(goal);



	ros::Rate loop_rate(200);
	
  	robot_OBJ.current_time = ros::Time::now();
  	robot_OBJ.last_time = ros::Time::now();

	Robot_enc_val enc_val_old = {{0,0,0,0},ros::Time::now()};
	while (ros::ok())
	{
		robot_OBJ.last_time = robot_OBJ.current_time;
		robot_OBJ.current_time = ros::Time::now();
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
					rc_motor_set(i+1, -m);
				}
			}
			else {
				rc_motor_set(1,0);
				rc_motor_set(2,0);
				rc_motor_set(3,0);
				rc_motor_set(4,0);
			}
		}



		if (use_MPU) {
			//http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom
			double dt = (robot_OBJ.current_time - robot_OBJ.last_time).toSec();

			//Calculater move change in x,y coordina. Angle is set based on gyro.
			robot_OBJ.vel_pose.a_z = robot_OBJ.MPU_data.dmp_TaitBryan[2];
			

			//ROS_INFO(" %f", robot_OBJ.vel_pose.a_z);
			//ROS_INFO(" %f", robot_OBJ.MPU_data.mag[1]);
			//ROS_INFO(" %f", robot_OBJ.robot_vel.actual_v[0]);
			//ROS_INFO(" %f", robot_OBJ.robot_vel.actual_v[1]);

			
			double dh = robot_OBJ.vel_pose.a_z - robot_OBJ.coordinate_ofset;
			//ROS_INFO(" %f", dh);
			double delta_x = (robot_OBJ.robot_vel.actual_v[0]*cos(dh) - robot_OBJ.robot_vel.actual_v[1]*sin(dh))*dt;
			double delta_y = (-robot_OBJ.robot_vel.actual_v[0]*sin(dh) + robot_OBJ.robot_vel.actual_v[1]*cos(dh))*dt;

			geometry_msgs::Quaternion odom_quat;
			//Save position changes to robot object
			robot_OBJ.vel_pose.x += delta_x;
			robot_OBJ.vel_pose.y += delta_y;

			//ROS_INFO(" %f", robot_OBJ.vel_pose.x);
			
			//Create quaternion from gyro + ofset for robot
			odom_quat = tf::createQuaternionMsgFromYaw(dh);

			//Pupulate odom_trans
			geometry_msgs::TransformStamped odom_trans;
			odom_trans.header.stamp = robot_OBJ.current_time;
			odom_trans.header.frame_id = "global";
			odom_trans.child_frame_id = "robot";

			odom_trans.transform.translation.x = robot_OBJ.vel_pose.x;
			odom_trans.transform.translation.y = robot_OBJ.vel_pose.y;
			odom_trans.transform.translation.z = 0.0;
			odom_trans.transform.rotation = odom_quat;

			//Brodcast transform
			odom_broadcaster.sendTransform(odom_trans);

			//next, we'll publish the odometry message over ROS
			nav_msgs::Odometry odom;
			odom.header.stamp = robot_OBJ.current_time;
			odom.header.frame_id = "global";

			//Set the position
			odom.pose.pose.position.x = robot_OBJ.vel_pose.x;
			odom.pose.pose.position.y = robot_OBJ.vel_pose.y;
			odom.pose.pose.position.z = 0.0;
			odom.pose.pose.orientation = odom_quat;

			//Set the velocity
			odom.child_frame_id = "robot";
			odom.twist.twist.linear.x = robot_OBJ.robot_vel.actual_v[0];
			odom.twist.twist.linear.y = robot_OBJ.robot_vel.actual_v[1];
			odom.twist.twist.angular.z = robot_OBJ.robot_vel.actual_v[2];
 
			//Publish the message
			odom_pub.publish(odom);
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
	

	return 0;
}