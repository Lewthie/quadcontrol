/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
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

bool goal_reached = true;

double roll, pitch, yaw;
double theta, psi, heading, diff;
double goal_roll, goal_pitch, goal_yaw;
double global_x,global_y,global_z, global_mag;
double head_x,head_y,head_z, head_mag;
double goal_x, goal_y, goal_z, goal_mag;
double dum_x, dum_y, dum_z, dum_mag;
double true_x, true_y, true_z, true_mag;
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

int main(int argc, char **argv)
{
    dum_x, dum_z = 1.0;
    dum_y = 0.0;
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber nav_goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10, goal_cb);
   	ros::Subscriber state_sub = nh.subscribe<nav_msgs::Odometry>("/ground_truth/state", 10, state_cb);
   	ros::Subscriber setpoints_sub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel_setpoint", 10, setpoints_cb);

    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(15.0);

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
        	 //   vel.header.frame_id = base_stabilized_frame_;
       	  //  vel.header.stamp = ros::Time::now();

            vel.linear.z = 0.0;
            vel.linear.x = velx_cmd;
            vel.linear.y = vely_cmd;
            
            vel.angular.x = 0.0;
            vel.angular.y = 0.0;
            vel.angular.z = wz_cmd;// - yaw ;
                
        }
      
      
          
     	local_vel_pub.publish(vel);


        // goal x y z - global x y z = heading x y z
        head_x = goal_x - global_x;
        head_y = goal_y - global_y;
        //head_z = goal_z - global_z;

        // true_x = head_x - dum_x;
        // true_x = head_y - dum_y;
        // true_x = head_z - dum_z;        

        // determine magnitudes of all three vectors
        // global_mag = sqrt(pow(global_x, 2.0) + pow(global_y, 2.0) + pow(global_z, 2.0));
        // goal_mag = sqrt(pow(goal_x, 2.0) + pow(goal_y, 2.0) + pow(goal_z, 2.0));
        


        //head_mag = sqrt(pow(head_x, 2.0) + pow(head_y, 2.0) + pow(head_z, 2.0));
        head_mag = sqrt(pow(head_x, 2.0) + pow(head_y, 2.0));



        // true_mag = sqrt(pow(true_x, 2.0) + pow(true_y, 2.0) + pow(true_z, 2.0));
        // dum_mag = sqrt(pow(dum_x, 2.0) + pow(dum_y, 2.0) + pow(dum_z, 2.0));

        // determine heading angle using cos law
        // heading = acos((pow(global_mag, 2.0) + pow(goal_mag, 2.0) + pow(head_mag, 2.0)) / (2.0 * global_mag * goal_mag));
        // theta = acos((pow(global_mag, 2.0) + pow(goal_mag, 2.0) - pow(head_mag, 2.0)) / (2.0 * global_mag * goal_mag));

        // don't forget to add the extra angle!!
        // theta = (pow(dum_mag, 2.0) + pow(head_mag, 2.0) - pow(true_mag, 2.0)) / (2.0 * dum_mag * head_mag);
        theta = acos(head_x / head_mag);
        if (head_y < 0.0) theta *= -1.0;

        //if (head_x > 0.0) theta += (M_PI / 4)

        diff = fabs(yaw) - fabs(theta);

        // Goal Debug Output

        // std::cout << " *** abs x: " << abs(goal_x - global_x) << " abs y: " << abs(goal_y - global_y) << " abs z: " << abs(goal_z - global_z) << " ***" <<std::endl;
        //std::cout << " *** heading = " << theta << std::endl;
        // std::cout << " *** goal x: " << goal_x << " goal y: " << goal_y << " goal z: " << goal_z << " ***"  << std::endl;
        // std::cout << " *** global x: " << global_x << " global y: " << global_y << " global z: " << global_z << " ***" << std::endl;
        // std::cout << " *** head x: " << head_x << " head y: " << head_y << " head z: " << head_z << " ***" << std::endl;
        // std::cout << " *** global mag: " << global_mag << std::endl;
        // std::cout << " *** goal mag: " << goal_mag << std::endl;
        // std::cout << " *** dum mag: " << dum_mag << std::endl;

        //std::cout << "quad roll: " << roll << " quad pitch: " << pitch << " quad yaw: " << yaw << std::endl;
        // std::cout << "quad roll: " << goal_roll << " quad pitch: " << goal_pitch << " quad yaw: " << goal_yaw << std::endl;
        std::cout << "yaw: \t\t\t" << yaw << std::endl;
        std::cout << "required heading: \t" << theta << std::endl;
        std::cout << "difference: \t\t" << fabs(diff) << std::endl;

        // look at current heading and required heading and turn in correct direction
        // keep it simple for now
        if (fabs(diff) > 0.08) wz_cmd = 0.1;
        else wz_cmd = 0.0;

        //if (head_mag > 0.1) velx_cmd = 0.1;
        //else velx_cmd = 0.0;

        //wz_cmd = 0.1;

        // if (abs(goal_x - global_x) > 0.0)
        // {
        //    if ((goal_x  global_x) < 0) 
        // }
        // else velx_cmd = 0.0;

        //while (!goal_reached)


        // if (abs(goal_y - global_y) > 0.0) vely_cmd = 0.2;
        // else vely_cmd = 0.0;

        // if (abs(goal_x - global_x) < 0.1 && abs(goal_y - global_y) < 0.1)
        // {
        //     velx_cmd = 0.0;
        //     vely_cmd = 0.0;
        // }
        // else
        // {
        //     if (goal_x < global_x) velx_cmd = -0.2;
        //     else if (goal_x > global_x) velx_cmd = 0.2;

        //     if (goal_y < global_y) vely_cmd = -0.2;
        //     else if (goal_y > global_y) vely_cmd = 0.2;    
        // }
        
        //if (abs(goal_z - global_z) > 0) velz_cmd = 0.2;
        //else velz_cmd = 0.0;
	
 //    // Status Output
	// std::cout<<"X: "<<global_x<< " Y: "<<global_y << " Z: "<<global_z<<std::endl;
	// std::cout<<"R: "<<roll << " P: "<< pitch  << " Y: "<<yaw <<std::endl;
	// std::cout<<"Vx: "<<current_state.twist.twist.linear.x<< " Vy: "<<current_state.twist.twist.linear.y  << " Vz: "<<current_state.twist.twist.linear.z <<std::endl;
	// std::cout<<"Wx: "<<current_state.twist.twist.angular.x<< " Wy: "<<current_state.twist.twist.angular.y  << " Wz: "<<current_state.twist.twist.angular.z <<std::endl;
	
	// std::cout<<"cmd vx: "<<velx_cmd<< " cmd vy: "<<vely_cmd<< " cmd wz: "<<wz_cmd - yaw<<std::endl;
	
	// std::cout<<"setpoint vx: "<<velx_cmd<< " setpoint vy: "<<vely_cmd<< " setpoint vz: "<<velz_cmd <<std::endl;
	// std::cout<<"setpoint wx: "<<wx_cmd<< " setpoint wy: "<<wy_cmd<< " setpoint wz: "<<wz_cmd <<std::endl;
	
	// std::cout<<"-----------------------------------------------\n"<<std::endl;



        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
