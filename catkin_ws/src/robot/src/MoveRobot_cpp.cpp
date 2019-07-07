#include <ros/ros.h>
#include "robot.h"

// MESSAGES
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <math.h>

#include <actionlib/server/simple_action_server.h>
#include <robot/MoveRobotAction.h>


class MoveRobotAction {
public:
    // CONSTRUCTIOR
    MoveRobotAction(std::string name) :   // In the argument of the constructor name of action is passed
        as_(nh_, name, false),      // Initialization list: _as...action server class is initialized with
                                    // provided parameters
        action_name_(name)           
    {   
        as_.registerGoalCallback(boost::bind(&MoveRobotAction::goalCB, this));
        as_.registerPreemptCallback(boost::bind(&MoveRobotAction::preemptCB, this));
        
        // Define topics to subscribe to
        odom_sub_ = nh_.subscribe("/odom", 1, &MoveRobotAction::controlCB, this);
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        as_.start();
    }

    void goalCB() {
        // Set goal position
        boost::shared_ptr<const robot::MoveRobotGoal> goal;
        
        goal = as_.acceptNewGoal();
        for(int i = 0; i < 3; i++) {
            target_position[i] = goal->target[i];
            
        }
        target_speed =  goal->speed;
        ROS_INFO("position: %f, speed: %f", target_position[0], target_speed);
    }

    void preemptCB() {
        // Set robot velocity to 0
        for(int i = 0; i < 3; i++) {
            feedback_.velocity[i] = 0;
        }
        as_.publishFeedback(feedback_);

        // Change action state to preempted
        as_.setPreempted();
    }

    void controlCB(const nav_msgs::Odometry::ConstPtr msg)
    {
        // First check that action is still active
        if (!as_.isActive())
        {
            return;
        }

        // Write recived data to robot_position
        robot_position[0] = msg->pose.pose.position.x;
        robot_position[1] = msg->pose.pose.position.y;
        tf::Quaternion q(msg->pose.pose.orientation.x,
                         msg->pose.pose.orientation.y,
                         msg->pose.pose.orientation.z,
                         msg->pose.pose.orientation.w);
        double roll, pitch, yaw;
        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
        robot_position[2] = yaw;

        double gl_vel[3];
        glob_velocity(gl_vel);
        
        rotate_velocities(robot_position[2], gl_vel, feedback_);

        
        vel_feedback_.linear.x = feedback_.velocity[0];
        vel_feedback_.linear.y = feedback_.velocity[1];
        vel_feedback_.angular.z = feedback_.velocity[2];

        cmd_vel_pub_.publish(vel_feedback_);
        for (int i = 0; i < 3; i++) {
            if (feedback_.velocity[i] != 0) {
                as_.publishFeedback(feedback_);
                return;
            }
            result_.final_position[i] = robot_position[i];
        }
        
        as_.setSucceeded(result_);


    }
protected:
    // Vars
    double target_position[3];
    double target_speed;
    double robot_position[3];
    double distance;


    // Msgs
    robot::MoveRobotFeedback feedback_;
    robot::MoveRobotResult result_;
    geometry_msgs::Twist vel_feedback_;

    // Subs
    ros::Subscriber odom_sub_;

    //Pub
    ros::Publisher cmd_vel_pub_;

    // Ros
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<robot::MoveRobotAction> as_;
    std::string action_name_;

    void glob_velocity(double *global_vel) {
        double ZERO_FACTOR = 100.;
        double P_koef = 0.5;

        // Velocity for each direction is calculated seperately
        for(int i = 0; i < 3; i++) {
            double dist;

            
            // Calculate distance to target
            dist = (target_position[i] - robot_position[i]);

            // Linear velocities are handled differently than angular
            if (i < 2) {

                // When robot is within 50mm of target position we stop moving.
                if (abs(dist) > 8) {
                    double sign, velocity;
                    sign = getSign<double>(dist);                           // Get sign of distance
                    velocity = ZERO_FACTOR + P_koef*abs(dist);              // Calculate velocity
                    velocity = limitNumber<float>(velocity, target_speed);  // Limit velocity with target speed
                    global_vel[i] = sign*velocity;                          // Asign to global_vel

                } else {
                    global_vel[i] = 0;
                }

            // Similar method for angular velocities -> different coeficients
            } else {
                if (abs(dist) > 0.03) {
                    double sign, velocity;
                    if(abs(dist)>M_PI) {
                        dist = dist - 2*M_PI;
                    }
                    
                    sign = getSign<double>(dist);
                    velocity = sign*0.3 + P_koef*dist;
                    velocity = limitNumber<float>(velocity, M_PI_2);
                    global_vel[i] = velocity;
                } else {
                    global_vel[i] = 0;
                }

            }
        }

        
    }

    void rotate_velocities(double angle, double *global_vel, robot::MoveRobotFeedback &loc_vel) {
        /*
        Input:  angle -> "Orientation of robot in GCS", 
                global_vel[3] -> "Required velocities in GCS"
                loc_vel -> "array where the calculates velocities in LCS will be stored"
        */
       /*
        if (global_vel[2] != 0) {
            angle = angle -getSign<double>(global_vel[2])*0.2;
        }
        */
        loc_vel.velocity[0] = cos(angle)* global_vel[0] + sin(angle) * global_vel[1];
        loc_vel.velocity[1] = - sin(angle)* global_vel[0] + cos(angle)* global_vel[1];
        loc_vel.velocity[2] = global_vel[2];
    }

};




int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_MoveRobot_node");

    MoveRobotAction moverobot(ros::this_node::getName());

    ros::spin();

    return 0;
}

