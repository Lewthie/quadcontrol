/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 *
 * Offboard Controller ROS Node
 * Luis Mejias, Falouthy Bahfenne
 * 2016 - 2017
 */

#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <tf/transform_datatypes.h>



//#include <hector_uav_msgs/TakeoffAction.h>
//#include <hector_uav_msgs/LandingAction.h>
//#include <hector_uav_msgs/PoseAction.h>
//#include <hector_uav_msgs/EnableMotors.h>
#include <actionlib/client/simple_action_client.h>




//opencv headers
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>



nav_msgs::Odometry current_state;
geometry_msgs::Twist vel_setpoint;
geometry_msgs::PoseStamped current_goal;

// bool goal_reached = true;
// bool turning, go = false;

double roll, pitch, yaw;
//double theta, psi, heading, diff;
double goal_roll, goal_pitch, goal_yaw;
double global_x,global_y,global_z, global_mag;
// double current_goal_x, current_goal_y;
// double head_x,head_y,head_z, head_mag;
double goal_x, goal_y, goal_z, goal_mag;
// double dum_x, dum_y, dum_z, dum_mag;
// double true_x, true_y, true_z, true_mag;
double velx_cmd, vely_cmd,velz_cmd;
double wx_cmd, wy_cmd, wz_cmd;
std::string base_link_frame_, base_stabilized_frame_, world_frame_;

ros::ServiceClient motor_enable_service_;

void state_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    current_state = *msg;
    
    //convert quaternions to euler
    tf::Quaternion q(current_state.pose.pose.orientation.x, current_state.pose.pose.orientation.y, current_state.pose.pose.orientation.z, current_state.pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	 
	 m.getRPY(roll, pitch, yaw);
	 
	//extract positions and copy to local position
	global_x = current_state.pose.pose.position.x;
	global_y = current_state.pose.pose.position.y;
	global_z = current_state.pose.pose.position.z;
		 
	 
	 
}

void setpoints_cb(const geometry_msgs::Twist::ConstPtr& msg)
{
    vel_setpoint = *msg;
    
   
	//extract velocities setpoints  and copy to local variables
	velx_cmd = vel_setpoint.linear.x;
	vely_cmd = vel_setpoint.linear.y;
	velz_cmd = vel_setpoint.linear.z;
	
	wx_cmd = vel_setpoint.angular.x;
	wy_cmd = vel_setpoint.angular.y;
	wz_cmd = vel_setpoint.angular.z;
		 
		 
	 
} 

void goal_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_goal = *msg;
    
    
    tf::Quaternion q(current_goal.pose.orientation.x, current_goal.pose.orientation.y, current_goal.pose.orientation.z, current_goal.pose.orientation.w);
    tf::Matrix3x3 m(q);
     
    m.getRPY(goal_roll, goal_pitch, goal_yaw);   

    goal_x = current_goal.pose.position.x;
    goal_y = current_goal.pose.position.y;
    goal_z = current_goal.pose.position.z;       
     
}

bool isAtGoal()
{
    // if (goal_x != global_x || goal_y != global_y || goal_z != global_z) return false;
    // return true;

    if (sqrt(pow(goal_x - global_x, 2.0) + pow(goal_y - global_y, 2.0)) > 0.2) return false;
    else return true;
}

double head_diff()
{
    double head_x,head_y,head_z, head_mag = 0.0;
    double theta = 0.0;
    double diff = 0.0;
    //bool turn = true;
    //std::cout << "TURNING!" << std::endl;
    // Determine Heading Vector (Quadrotor to Target)
    head_x = goal_x - global_x;
    head_y = goal_y - global_y;

    head_mag = sqrt(pow(head_x, 2.0) + pow(head_y, 2.0));

    theta = acos(head_x / head_mag);
    if (head_y < 0.0) theta *= -1.0;

    diff = fabs((fabs(yaw) - fabs(theta)));

    std::cout << "yaw: \t\t\t" << yaw << std::endl;
    std::cout << "required heading: \t" << theta << std::endl;
    std::cout << "difference: \t\t" << diff << std::endl;

    return diff;       
}

double turn_state()
{

    double diff = 0.0;

    diff = head_diff();

    if (fabs(diff) > 0.08) return 0.1;
    else return 0.0;

}

double go_state()
{
    // if heading not correct OR distance is less than 0.2, return 0
    if (head_diff() > 0.08) return 0.0;

    // else move return 0.2
    else return 0.2; 
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber nav_goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10, goal_cb);
   	ros::Subscriber state_sub = nh.subscribe<nav_msgs::Odometry>("/ground_truth/state", 10, state_cb);
   	ros::Subscriber setpoints_sub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel_setpoint", 10, setpoints_cb);

    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(15.0);

    goal_x = global_x;
    goal_y = global_y;
    goal_z = global_z;

// #ifdef HECTOR
    // wait for connection to the simulator, and wait for ten packages to arrive
    while(ros::ok() && current_state.header.seq > 10)
    {
        ros::spinOnce();
        rate.sleep();
    }

    //define fixed velocity commands to be sent to the sim
    geometry_msgs::Twist vel;
    //  vel.header.stamp = ros::Time::now();

    
    vel.linear.x = 0.0;
    vel.linear.y = 0.0;
    vel.linear.z = 0.2;
    vel.angular.x = 0.0;
    vel.angular.y = 0.0;
    vel.angular.z = 0.0; 

    while(ros::ok())
    {

	// vel.header.frame_id = base_stabilized_frame_;
   	// vel.header.stamp = ros::Time::now();

    	if (global_z >= 1.0)
        {
          //  vel.header.frame_id = base_stabilized_frame_;
       	  //  vel.header.stamp = ros::Time::now();

            vel.linear.z = 0.0;
            vel.linear.x = velx_cmd;
            vel.linear.y = vely_cmd;
            
            vel.angular.x = 0.0;
            vel.angular.y = 0.0;
            vel.angular.z = wz_cmd;// - yaw ;
                
        }


        if (!isAtGoal())
        {

            wz_cmd = turn_state();
            velx_cmd = go_state();

        } 
        else
        {
            wz_cmd = 0.0;
            velx_cmd = 0.0;
        }
        
        local_vel_pub.publish(vel);


        // *** DEBUG STUFF - DO NOT DELETE ***

        // std::cout << " *** abs x: " << abs(goal_x - global_x) << " abs y: " << abs(goal_y - global_y) << " abs z: " << abs(goal_z - global_z) << " ***" <<std::endl;
        // std::cout << " *** heading = " << theta << std::endl;
        std::cout << " *** goal x: " << goal_x << " goal y: " << goal_y << " goal z: " << goal_z << " ***"  << std::endl;
        std::cout << " *** global x: " << global_x << " global y: " << global_y << " global z: " << global_z << " ***" << std::endl;
        // // std::cout << " *** head x: " << head_x << " head y: " << head_y << " head z: " << head_z << " ***" << std::endl;
        // // std::cout << " *** global mag: " << global_mag << std::endl;
        // // std::cout << " *** goal mag: " << goal_mag << std::endl;
        // // std::cout << " *** dum mag: " << dum_mag << std::endl;



        //std::cout << "yaw: \t\t\t" << yaw << std::endl;
        //std::cout << "required heading: \t" << theta << std::endl;
        // std::cout << "difference: \t\t" << diff << std::endl;

        // *** DEBUG STUFF - DO NOT DELETE ***        

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
