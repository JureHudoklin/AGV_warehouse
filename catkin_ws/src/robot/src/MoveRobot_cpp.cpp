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
        robot_position[1] = msg->pose.pose.position.x;
        tf::Quaternion q(msg->pose.pose.orientation.x,
                         msg->pose.pose.orientation.y,
                         msg->pose.pose.orientation.z,
                         msg->pose.pose.orientation.w);
        double roll, pitch, yaw;
        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
        robot_position[2] = yaw;

        get_distance();

        double gl_vel[3];
        glob_velocity(gl_vel);
        MoveRobotAction::rotate_velocities(robot_position[3], gl_vel, feedback_);

        if (distance < 20) {
            for(int i = 0; i < 3; i++) {
                feedback_.velocity[i] = 0;
            }
            as_.publishFeedback(feedback_);
            as_.setSucceeded(result_);
        }

        as_.publishFeedback(feedback_);
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

    // Subs
    ros::Subscriber odom_sub_;

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

        angle = atan((target_position[1] - robot_position[1] ) / (target_position[0] - robot_position[0]));
        
        global_vel[0] = cos(angle)*vel;
        global_vel[1] = sin(angle)*vel;

        if (target_position[2] > robot_position[2]) {
            global_vel[2] = M_PI_4;
        } else {
            global_vel[2] = -M_PI_4;
        }
    }

    void rotate_velocities(double angle, double global_vel[3], robot::MoveRobotFeedback &loc_vel) {
        loc_vel.velocity[0] = cos(angle)* global_vel[0] - sin(angle)* global_vel[1];
        loc_vel.velocity[1] = sin(angle)* global_vel[1] - cos(angle)* global_vel[0];
        loc_vel.velocity[2] = global_vel[2];
    }

};




int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_MoveRobot_node");

    MoveRobotAction moverobot(ros::this_node::getName());

    ros::spin();

    return 0;
}

