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
struct Robot_state_struct {
	std::string ID_s;
	bool state_s;
    bool action_s;
	double[3] vel_s;
	double[3] move_s;
};

//Classes
class Robot_control {
	private:
		std::vector<Robot_state_struct> robot_history;
	public:
		void robot_state_callback(const);
        void robot_next_action();
};
void Robot_control::robot_state_callback(const robot::Control &control_msg) {
	/* Fucntion is called when new robot_state_msg is recieved.
	Based on the id of message and message history robot_history of class is updated.
	 */
	int a = 0;

	for (int i = 0; i < robot_history.size(); i++) {	//Iterates through all previous states
		if (robot_history[i].ID_s.compare(control_msg.ID) == 0) {	//If ID was already recorded.
			robot_history.erase(robot_history.begin()+i);	//Id is deleted
			robot_history.push_back({contorl_msg.ID, contorl_msg.state, contorl_msg.acction_s, contorl_msg.vel, contorl_msg.move});	//And moved to the back
			a = 1;
		}
	}

	if (a == 0) {	//ID does not yet exist
		robot_history.push_back({contorl_msg.ID, contorl_msg.state, contorl_msg.vel, contorl_msg.move});
	}

	return;
}
void robot_next_action() {

    int a = -1;
    for (int i = 0; i < robot_history.size(); i++) {
        if (roboot_history[i].state == false) {
            return;
        }
        else if(roboot_history[i].action == true) {
            if (a == -1) {
                a = i;
            }
        }
        return;
    }

    
}

//Publishers
ros::Publisher cmd_vel_pub;

//Subscribers
ros::Subscriber robot_state_sub;
ros::Subscriber robot_encoders_sub;

//Define used variable
int use_motors;
double wheel_diameter, hub_radius;

int main(int argc, char** argv) {

    //Definer and run the node
    ros::init(argc, argv, "robot_logic");
	ros::NodeHandle nh;

    //Set node parameters
    nh.getParam("/robot_node/use_motors", use_motors);
    nh.getParam("/robot_logic_node/wheel_diameter", wheel_diameter);
    nh.getParam("/robot_logic_node/hub_radius", hub_radius);

    robot_state_sub = nh.subscribe("/robot_state", 100, &robot_state_callback);

    if (use_motors == 1) {
        ROS_INFO("Initialize encoders");
        if(rc_encoder_init() == -1)	{
			ROS_ERROR_STREAM("Encoder initialization unsucessfull");
		}
    }

    //Set time variables and ros loop rate
    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    ros::Rate loop_rate(10);

    while (ros::ok())
	{
        current_time = ros::Time::now();


		ros::spinOnce();    //On spin it checks messages
		loop_rate.sleep();
	}
    if (use_motors == 1) {
        rc_encoder_cleanup();
    }
	return 0;
}