#include <ros/ros.h>

// MESSAGES
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

#include <actionlib/server/simple_action_server.h>
#include <robot/FollowLineAction.h>


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
        line_sen_sub_ = nh_.subscribe("/line_sen", 1, &FollowLineAction::controlCB, this);
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
        ROS_INFO("Moving robot to square: %d, %d", target_square[0], target_square[1]);
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

    void controlCB(const robot::Line_sensor::ConstPtr msg)
    {
        // First check that action is still active
        if (!as_.isActive())
        {
            return;
        }

        // Write recived data to robot_position
        
        
        as_.setSucceeded(result_);


    }
protected:
    // Vars
    int target[2];
    double target_speed;  

    // Msgs
    robot::FollowLineFeedback feedback_;
    robot::FollowLineResult result_;

    // Subs
    ros::Subscriber line_sen_sub_;

    //Pub
    ros::Publisher cmd_vel_pub_;

    // Ros
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<robot::FollowLineAction> as_;
    std::string action_name_;
};






int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_FollowLine_node");

    FollowLineAction followline(ros::this_node::getName());

    ros::spin();

    return 0;
}
