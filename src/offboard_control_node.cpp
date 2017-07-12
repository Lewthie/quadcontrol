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
#include <vector>
#include <fstream>
#include <iostream>
#include <numeric>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/LaserScan.h>
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

#define deg2rad(a) (a * (M_PI / 180.0))
#define rad2deg(a) (a * (180.0 / M_PI))
#define sgn_diff(a,b) atan2(sin(a - b), cos(a - b))

nav_msgs::Odometry current_state;
geometry_msgs::Twist vel_setpoint;
geometry_msgs::PoseStamped current_goal;
sensor_msgs::LaserScan laser_scan;

// bool goal_reached = true;
// bool turning, go = false;

// debug outputs for corridors


double roll, pitch, yaw;
// double range[720];
//std::vector<double> range;
double range[180];
double goal_roll, goal_pitch, goal_yaw;
double global_x,global_y,global_z;
double goal_x, goal_y, goal_z;
double velx_cmd, vely_cmd,velz_cmd;
double wx_cmd, wy_cmd, wz_cmd;
double old_goal_x, old_goal_y;
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

void laser_cb(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    laser_scan = *msg;
    
    // this is dumb and inefficient; find a way to alter the msg so we don't have to do this
    // min_ang, max_ang and increments (or whatever they're called) has to be changed from the msg
    //range.clear();
    int j = 0;
    // 
    for (int i = 179; i < 900; i = i + 4)
    {
        // range.push_back(laser_scan.ranges[i]);
        range[j] = laser_scan.ranges[i]; 
        j++;
        // std::cout << "range[" << j << "] : " << range[j] << std::endl;
    }

    //range.assign(laser_scan.ranges);
     
}

// bool isAtGoal()
// {
//     return (sqrt(pow(goal_x - global_x, 2.0) + pow(goal_y - global_y, 2.0)) < 0.2);    
// }

double distToGoal()
{
    return sqrt(pow(goal_x - global_x, 2.0) + pow(goal_y - global_y, 2.0));
}

bool goalSet()
{
    //std::cout << "New Goal is Set" << std::endl;
    return (goal_x != old_goal_x && goal_y != old_goal_y);
}

double turn_state()
{

    // Determines correct direction of yaw

    double head_x,head_y,head_z, head_mag = 0.0;
    double theta = 0.0;

    // Determine Heading Vector (Quadrotor to Target)
    head_x = goal_x - global_x;
    head_y = goal_y - global_y;

    head_mag = sqrt(pow(head_x, 2.0) + pow(head_y, 2.0));

    theta = acos(head_x / head_mag);
    if (head_y < 0.0) theta *= -1.0; 

    // calculate signed difference
    // return atan2(sin(theta - yaw), cos(theta - yaw));
    return sgn_diff(theta, yaw);

}

//  
// double get_scan()
// {

//     double rgOut[180];
//     double min;

//     for (int i = 0; i < 180; i++)
//     {
//         //min = yaw - (89 - i);
//         rgOut[i] = (yaw * (180 / M_PI)) - (89 - i);
//         //std::cout << i << " : " << rgOut[i] << ", yaw: " << yaw * 180/M_PI << std::endl; 
//     }

//     return 0.0;
// }

// double convertYaw(double yaw_in)
// {
//     // yaw_in = fmod(yaw_in + 180.0,360.0);
//     // if (yaw_in < 0.0)
//     //     yaw_in += 360.0;
//     // return yaw_in - 180.0;
//     return yaw_in;
// }

double pathfinder(double rng_a [], double dir)
{

    // // Upgraded turn_state function. 

    // double head_x,head_y,head_z, head_mag = 0.0;
    // double theta = 0.0;
    // double start, finish;
    int j = 0;
    double mv_avg;
    double mid = 30;
    double avg_den,sum, avg_n = 0.0;
    double start, finish;
    double diff = 100.0;
    // std::vector<double> mid;
    bool corr = false;

    double rng_ang[180];
    double min, temp, temp2, temp3, path;

    // find world frame angles in scan range
    for (int i = 0; i < 180; i++)
    {
        // rng_ang[i] = rad2deg(yaw) - (89 - i);
        // rng_ang[i] = convertYaw(rad2deg(yaw) + (89 - i));
        rng_ang[i] = rad2deg(yaw) + (89 - i);
    }

    for (int i = 0; i < 180; i++)
    {
        if (rng_a[i] < 25 && rng_a[i] > 0.1)
        {
            
            sum += rng_a[i];
            avg_den++;

        }
    }

    // determine average scan value
    // this value will be used to determine the presence of corridors
    avg_n = sum / avg_den;

    // iterate through scanned range
    for (int i = 1; i < 179; i++)
    {

        // calculate moving average 
        mv_avg = (rng_a[i] + rng_a[i - 1] + rng_a[i + 1]) / 3.0;

        // determine position of corridors -- maybe handy to have debug outputs for corridor positions
        if (corr == false)
        {
            if (mv_avg > avg_n && rng_a[i - 1] < avg_n)
            {
                //start = rng_ang[i];
                start = i;
                corr = true;
                // std::cout << "start " << start << " : " << rng_ang[i] << "VVVVVVVVVVVVVVV" << std::endl;
            }
        }
        else 
        {
            if (mv_avg < avg_n && rng_a[i - 1] > avg_n)
            {
                //finish = rng_ang[i];
                finish = i;

                mid = start + (finish - start) / 2.0;
                temp = rng_ang[int (round(mid))];

                // pick direction closest to corridor
                if (fabs(rad2deg(dir) - temp) < diff)
                {
                    diff = fabs(rad2deg(dir) - temp);
                    path = temp;
                }

                // Debug Lines
                // std::cout << "******************\n\nfinish : " << finish << " : " << rng_ang[i] << std::endl;
                // std::cout << "angle for mid : " << temp << std::endl;
                // std::cout << "dir : " << rad2deg(dir) << "\n\n^^^^^^^^^^^^^^^^^^^^" << std::endl;

                corr = false;
            }
        }
    }   

    return deg2rad(path);    


}

int main(int argc, char **argv)
{

    std::ofstream myFile;
    double diff = 0.0;
    double path_way;
    bool go = false;
    bool centre = false;
    int j = 0;

    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    // Subscribe to relevant topics
    ros::Subscriber nav_goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10, goal_cb); // nav goal
   	ros::Subscriber state_sub = nh.subscribe<nav_msgs::Odometry>("/ground_truth/state", 10, state_cb);              // ground truth pos
   	ros::Subscriber setpoints_sub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel_setpoint", 10, setpoints_cb);      // velocity
    ros::Subscriber laser_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan/", 10, laser_cb);                       // laser scan readings

    // Publisher
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);                               // velocity commands

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(15.0);

    // wait for connection to the simulator, and wait for ten packages to arrive
    while(ros::ok() && current_state.header.seq > 10)
    {
        ros::spinOnce();
        rate.sleep();
    }

    //define fixed velocity commands to be sent to the sim
    geometry_msgs::Twist vel;
    
    vel.linear.x = 0.0;
    vel.linear.y = 0.0;
    vel.linear.z = 0.2;
    vel.angular.x = 0.0;
    vel.angular.y = 0.0;
    vel.angular.z = 0.0;

    while(ros::ok())
    {

        // ensure appropriate altitude
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
   


        // Pathfinding **************************************************************

        // Fix this. Nav algorithm should be focus.

        // calculate difference between current heading (yaw) and goal heading
        diff = turn_state();

        if (goalSet()) // find if new goal set
        {
            centre = false;
            std::cout << "New Goal Set!" << std::endl;
        }
        else if (!centre) // otherwise, check if facing goal heading
        {
            if (fabs(diff) >= 0.08)
            {
                velx_cmd = 0.0;
                wz_cmd = (diff > 0.0) ? 0.1 : -0.1;
            }
            else
            {
                velx_cmd = 0.0;
                wz_cmd = 0.0;
                centre = true;
                std::cout << "Centred on Goal Coordinates" << std::endl;
            }
        }
        else if (distToGoal() > 0.2 && centre) // if facing goal heading, move forward
        {
            if (range[89] >= distToGoal()) // if no obstacles, proceed
            {
                
                std::cout << "Forward path unobstructed - Proceed to Goal" << std::endl;
                velx_cmd = 0.2;
                wz_cmd = 0.0;

            }
            else // else, use pathfinder function
            {
                std::cout << "Obstacles Detected - Using Pathfinder..." << std::endl;
                path_way = pathfinder(range, yaw + diff);

                if (fabs(sgn_diff(path_way,yaw)) >= 0.08)
                //if (fabs(diff) >= 0.08)
                {
                    // std::cout << "Turning; Current: " <<  << ". Turning to " << rad2deg(fabs(sgn_diff(path_way,yaw))) << std::endl;
                    std::cout << "Turning; Current: " << rad2deg(yaw) << ". Turning to " << rad2deg(path_way) << std::endl;
                    velx_cmd = 0.0;
                    wz_cmd = (sgn_diff(path_way,yaw) > 0.0) ? 0.1 : -0.1;
                    //wz_cmd = (diff > 0.0) ? 0.1 : -0.1;
                }
                else
                {
                    std::cout << "Advancing..." << std::endl;
                    velx_cmd = 0.2;
                    wz_cmd = 0.0;
                }
            }
  
        }
        else if (distToGoal() < 0.2) // if goal reached, idle
        {
            std::cout << "Standby..." << std::endl;
            velx_cmd = 0.0;
            wz_cmd = 0.0;
        }

        // store goal x and y in a temp variable
        old_goal_x = goal_x;
        old_goal_y = goal_y;

        //***********************************************************************************

        // Publish velocity msgs
        local_vel_pub.publish(vel);


        // *** DEBUG STUFF - DO NOT DELETE ***

        std::cout << "\n*******************" << std::endl;
        std::cout << "path = " << rad2deg(sgn_diff(path_way, yaw)) << std::endl;
        std::cout << "direction = " << rad2deg(yaw + diff) << std::endl;
        // std::cout << "converted yaw = " << convertYaw(rad2deg(yaw)) << std::endl;
        std::cout << "yaw = " << rad2deg(yaw) << std::endl;
        std::cout << "*******************\n" << std::endl;

        // std::cout << " *** abs x: " << abs(goal_x - global_x) << " abs y: " << abs(goal_y - global_y) << " abs z: " << abs(goal_z - global_z) << " ***" <<std::endl;
        // std::cout << " *** heading = " << theta << std::endl;
        // std::cout << " *** goal x: " << goal_x << " goal y: " << goal_y << " goal z: " << goal_z << " ***"  << std::endl;
        // std::cout << " *** global x: " << global_x << " global y: " << global_y << " global z: " << global_z << " ***" << std::endl;
        // std::cout << " *** head x: " << head_x << " head y: " << head_y << " head z: " << head_z << " ***" << std::endl;
        // std::cout << " *** global mag: " << global_mag << std::endl;
        // std::cout << " *** goal mag: " << goal_mag << std::endl;
        // std::cout << " *** dum mag: " << dum_mag << std::endl;
        // std::cout << isAtGoal() << std::endl;

        //for (int i = 0; i < 300; i++)
        //{
        //    std::cout << " *** [" << i << "] : "  << range[i] << " metres. ***" << std::endl; 
        //}

        //j = 0;
        // std::cout << "************** START ********************" << std::endl;
        // for (std::vector<double>::iterator it = range.begin() ; it != range.end(); ++it)
        // {
        //     std::cout << *it << " metres. ***" << std::endl;
        //     //j++; 
        // }
        // std::cout << "******************* END ********************" << std::endl;
        //j = 0;
        //myFile.close();

        // *** DEBUG STUFF - DO NOT DELETE ***        

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
