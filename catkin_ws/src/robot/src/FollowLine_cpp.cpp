#include <ros/ros.h>

// MESSAGES
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <robot/Line_sensor.h>
#include <math.h>

#include <dynamic_reconfigure/server.h>
#include <robot/ReconfigureLineConfig.h>

#include <actionlib/server/simple_action_server.h>
#include <robot/FollowLineAction.h>

#define DISTANCE_BETWEEN_SENSORS 9
#define K_SIDE 4
#define K_ROTATE 0.045

template <class T>
int getMinIndex(T* number, int size) {

    int index = 0;
    T a = number[0];
    for(int i = 1; i<size; i++) {
        //ROS_INFO("%f",number[i]);
        if (number[i]<a) {
            a = number[i];
            index = i;
        }
    }
    return index;
}


template <class T>
T getSign(T number) {
    if (number < 0) {
        return -1;
    } else {
        return 1;
    }
}

template <class T>
T limitNumber(T number, T max) {
    if (number < max) {
        return number;
    } else {
        return max;
    }
}

double P_SIDE;
double D_SIDE;
double P_ROTATE;

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

    void controlCB(const robot::Line_sensor::ConstPtr msg)
    {
        // First check that action is still active
        if (!as_.isActive())
        {
            return;
        }

        // Calculate line postion
        double* line_position;
        double fr_sen[8];
        double ba_sen[8];
        //ROS_INFO("test");
        //ROS_INFO("%f,  %f", msg->front_sensors[0], fr_sen[0]);
        for(int i = 0; i<8; i++) {
            fr_sen[i] = msg->front_sensors[i];
            //ROS_INFO("%f,  %f", msg->front_sensors[i], fr_sen[i]);
            ba_sen[i] = msg->back_sensors[i];
        }
        line_position = calculateLinePosition(fr_sen, ba_sen);
        
        ROS_INFO("pozicija_crte: front-> %f, back-> %f", line_position[0], line_position[1]);



        geometry_msgs::Twist twist;
        double error;
        error = line_position[0] + line_position[1];


        
        
        twist.linear.x = target_speed; //getSign<double>(target_speed) * (abs(target_speed) - abs(line_position[0]) - abs(line_position[1]));
        twist.linear.y = limitNumber<double>(error*P_SIDE + (error - error_old) * D_SIDE, 300.);
        twist.angular.z = limitNumber<double>((line_position[0] - line_position[1])*P_ROTATE, 4.);

        cmd_vel_pub_.publish(twist);
        
        delete[] line_position;
        //as_.setSucceeded(result_);
    }

    
protected:
    // Vars
    int target[2];
    double error_old = 0;
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

    double* calculateLinePosition(double* front_sen, double* back_sen) {
        double* line_pos = new double[2];

        int f = getMinIndex<double>(front_sen+1, 6)+1;
        int b = getMinIndex<double>(back_sen+1, 6)+1;

        ROS_INFO("f. %d, b: %d", f,b);
        double f_v, b_v;


        //f_v = (f-3.5) - (0.57-0.33*front_sen[f-1]) + (0.57-0.33*front_sen[f+1]);

        f_v = -(f-3.5) - 0.33*(front_sen[f-1] - front_sen[f+1]);    
        b_v = -(b-3.5) - 0.33*(back_sen[b-1] - back_sen[b+1]);

        line_pos[0] = DISTANCE_BETWEEN_SENSORS * f_v;

        line_pos[1] = DISTANCE_BETWEEN_SENSORS * b_v;

        return line_pos;
    }
};


void reconfigure_callback(robot::ReconfigureLineConfig &config, uint32_t level) {
    P_SIDE = config.P_koef;
    D_SIDE = config.D_koef;
    P_ROTATE = config.P_rotate_koef;
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
