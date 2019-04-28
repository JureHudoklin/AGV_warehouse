#include <ros/ros.h>

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
        //ROS_INFO(" %f, %f, %f", robot_position[0], robot_position[1], robot_position[2]);

        get_distance();

        double gl_vel[3];
        glob_velocity(gl_vel);
        //ROS_INFO("global vel:  %f, %f", gl_vel[0], gl_vel[1]);
        rotate_velocities(robot_position[2], gl_vel, feedback_);

        //ROS_INFO("local vel:  %f, %f", feedback_.velocity[0], feedback_.velocity[1]);
        vel_feedback_.linear.x = feedback_.velocity[0];
        vel_feedback_.linear.y = feedback_.velocity[1];
        vel_feedback_.angular.z = feedback_.velocity[2];


        if (distance < 40 && feedback_.velocity[2] == 0) {
            for(int i = 0; i < 3; i++) {
                feedback_.velocity[i] = 0;
            }
            vel_feedback_.linear.x = feedback_.velocity[0];
            vel_feedback_.linear.y = feedback_.velocity[1];
            vel_feedback_.angular.z = feedback_.velocity[2];
            as_.publishFeedback(feedback_);
            as_.setSucceeded(result_);
        } else if(distance < 40) {
            for(int i = 0; i < 2; i++) {
                feedback_.velocity[i] = 0;
            }
            vel_feedback_.linear.x = feedback_.velocity[0];
            vel_feedback_.linear.y = feedback_.velocity[1];
            vel_feedback_.angular.z = feedback_.velocity[2];
            as_.publishFeedback(feedback_);
        } else {
            vel_feedback_.linear.x = feedback_.velocity[0];
            vel_feedback_.linear.y = feedback_.velocity[1];
            vel_feedback_.angular.z = feedback_.velocity[2];
            as_.publishFeedback(feedback_);
        }
        cmd_vel_pub_.publish(vel_feedback_);

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

    void get_distance(void) {
        double sum = 0;
        for(int i = 0; i < 2; i++) {
            sum += pow(robot_position[i] - target_position[i], 2);
        }
        distance = sqrt(sum);
    }

    void glob_velocity(double *global_vel) {
        double vel, angle;
        if (distance<100) {
            vel = 200;
        } else {
            vel = target_speed;
        }

        double delta_x, delta_y;
        delta_y = (target_position[1] - robot_position[1]);
        delta_x = (target_position[0] - robot_position[0]);

        if (delta_x < 1. && delta_x >= 0) {
            angle = M_PI/2;
        } else if(delta_x > -1. && delta_x < 0) {
            angle = -M_PI/2;
        } else {
            ROS_INFO("TLE je, %f", delta_x);
            angle = atan(delta_y / delta_x);
        }
        
        global_vel[0] = cos(angle)*vel;
        global_vel[1] = sin(angle)*vel;

        if (target_position[2] - 0.1 > robot_position[2]) {
            global_vel[2] = M_PI/2;      //TUKAJ DAJ NEK PID KONTROLER ZA HITROST VRTENJA
        } else if(target_position[2] + 0.1 < robot_position[2]){
            global_vel[2] = -M_PI/2;
        }
    }

    void rotate_velocities(double angle, double *global_vel, robot::MoveRobotFeedback &loc_vel) {
        /*
        Input:  angle -> "Orientation of robot in GCS", 
                global_vel[3] -> "Required velocities in GCS"
                loc_vel -> "array where the calculates velocities in LCS will be stored"
        */
        
        loc_vel.velocity[0] = cos(angle)* global_vel[0] - sin(angle)* global_vel[1];
        loc_vel.velocity[1] = -sin(angle)* global_vel[0] + cos(angle)* global_vel[1];
        loc_vel.velocity[2] = global_vel[2];
    }

};




int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_MoveRobot_node");

    MoveRobotAction moverobot(ros::this_node::getName());

    ros::spin();

    return 0;
}

