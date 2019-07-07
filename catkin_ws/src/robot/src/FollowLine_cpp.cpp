#include <ros/ros.h>
#include "robot.h"

// MESSAGES
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PolygonStamped.h>
#include <robot/Line_sensor.h>
#include <math.h>

#include <dynamic_reconfigure/server.h>
#include <robot/ReconfigureLineConfig.h>

#include <actionlib/server/simple_action_server.h>
#include <robot/FollowLineAction.h>

#define K_SIDE 4
#define K_ROTATE 0.045



double P_SIDE;
double D_SIDE;
double P_ROTATE;
double D_ROTATE;

class FollowLineAction {
public:
    // CONSTRUCTIOR
    FollowLineAction(std::string name) :   // In the argument of the constructor name of action is passed
        as_(nh_, name, false),              // Initialization list: _as...action server class is initialized with
                                            // provided parameters
        action_name_(name)           
    {   
        as_.registerGoalCallback(boost::bind(&FollowLineAction::goalCB, this));
        as_.registerPreemptCallback(boost::bind(&FollowLineAction::preemptCB, this));
        
        // Define topics to subscribe to
        line_sen_pos_sub_ = nh_.subscribe("/line_pos", 1, &FollowLineAction::controlCB, this);
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        as_.start();
    }

    void goalCB() {
        // Set goal position
        boost::shared_ptr<const robot::FollowLineGoal> goal;
        
        goal = as_.acceptNewGoal();
        for(int i = 0; i < 2; i++) {
            target[i] = goal->target_square[i];
            
        }
        target_speed =  goal->speed;
        ROS_INFO("Moving robot to square: %d, %d", target[0], target[1]);
    }

    void preemptCB() {
        // Set robot velocity to 0
        geometry_msgs::Twist twist;

        twist.linear.x = 0;
        twist.linear.y = 0;
        twist.angular.z = 0;

        cmd_vel_pub_.publish(twist);

        // Change action state to preempted
        as_.setPreempted();
    }

    void controlCB(const geometry_msgs::PolygonStamped::ConstPtr msg)
    {
        // First check that action is still active
        if (!as_.isActive())
        {
            return;
        }

        geometry_msgs::Twist twist;
        double error_side, error_rotate;

        // Multiplying by 1000 is done to transform to mmilimeters. If everything is changed to meters this can be removed
        error_side = msg->polygon.points[0].y*1000 +  msg->polygon.points[3].y*1000;
        error_rotate = msg->polygon.points[0].y*1000 - msg->polygon.points[3].y*1000;
        
        twist.linear.x = target_speed; //getSign<double>(target_speed) * (abs(target_speed) - abs(line_position[0]) - abs(line_position[1]));
        twist.linear.y = limitNumber<double>(error_side*P_SIDE + (error_side - error_side_old) * D_SIDE, 300.);
        twist.angular.z = limitNumber<double>(error_rotate*P_ROTATE + (error_rotate- error_rotate_old) * D_ROTATE, 4.);

        cmd_vel_pub_.publish(twist);
        error_side_old = error_side;
        error_rotate_old = error_rotate;

        //as_.setSucceeded(result_);
    }

    
protected:
    // Vars
    int target[2];
    double error_side_old = 0;
    double error_rotate_old = 0;
    double target_speed;

    // Msgs
    robot::FollowLineFeedback feedback_;
    robot::FollowLineResult result_;

    // Subs
    ros::Subscriber line_sen_pos_sub_;

    //Pub
    ros::Publisher cmd_vel_pub_;

    // Ros
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<robot::FollowLineAction> as_;
    std::string action_name_;

};


void reconfigure_callback(robot::ReconfigureLineConfig &config, uint32_t level) {
    P_SIDE = config.P_koef;
    D_SIDE = config.D_koef;
    P_ROTATE = config.P_rotate_koef;
    D_ROTATE = config.D_rotate_koef;
	return;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_FollowLine_node");

    FollowLineAction followline(ros::this_node::getName());

    // Set dynamic reconfigure
	dynamic_reconfigure::Server<robot::ReconfigureLineConfig> server;
	dynamic_reconfigure::Server<robot::ReconfigureLineConfig>::CallbackType rec_f;
	rec_f = boost::bind(&reconfigure_callback, _1, _2);
	server.setCallback(rec_f);   

    ros::spin();

    return 0;
}
