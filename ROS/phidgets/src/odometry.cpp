/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Phidgets based wheel odometry for a differential drive system
 *  For use with Phidgets PN 1047_1 , High Speed 4 port encoder. 
 *  See http://www.ros.org/wiki/navigation/Tutorials/RobotSetup/Odom
 *  
 * 	Some structural content borrowed from Bob Mottram's work documented 
 *  here: http://wiki.ros.org/phidgets
 *  
 *  Additionally the d	iff_tf node from the differential_drive package
 *  documented here was referenced throughout: http://wiki.ros.org/differential_drive
 * 
 * 	Remaining work courtesy of Mike Gallagher
 *  e-mail: mjg@152.case.edu
 * 
 *  All rights reserved.
 * 
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <stdio.h>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "phidgets/encoder_params.h"
#include <std_msgs/Bool.h>

// used to prevent callbacks from accessing variables
// before they are initialised
bool initialised = false;

// index numbers of left and right encoders for the case
// where two or more encoders exist on the same Phidget device
int encoder_index_left=-1;
int encoder_index_right=-1;

// should we announce every encoder count that arrives?
bool verbose = false;

// normally on a differential drive system to when moving
//  forwards the wheels are rotating in opposite directions
int encoder_direction_left = -1;
int encoder_direction_right = 1;

// distance between the wheel centres
double wheelbase_mm = 400;

// encoder counts
int right_encoder_diff = 0;
int left_encoder_diff = 0;
int left_prev_encoder = 0;
int right_prev_encoder = 0;
int left_encoder_count = 0;
int right_encoder_count = 0;


// keep track if the initial encoder values so that relative
// movement can be reported
int start_encoder_count_left = 0;
int start_encoder_count_right = 0;

// encoder counts per millimetre
double left_encoder_counts_per_mm = 0;
double right_encoder_counts_per_mm = 0;

ros::Subscriber left_encoder_sub;
ros::Subscriber right_encoder_sub;
ros::Subscriber encoders_sub;

ros::Publisher odom_reset_pub;

bool g_odom_reset = false;

// pose estimate
double x = 0.0;
double y = 0.0;
double theta = 0.0;
double D = 0.0;

// velocity estimate
double vx = 0.1;
double vy = -0.1;
double vtheta = 0.1;
double vD = 0.1;

double rotation_offset=0;

//Fairly certain these phidgets encoders will wrap around after 32 bits 
// of signed integer storage. So if we give 1 bit to signing, then wrap around
//will occur at 2^31. 

int encoder_max =2147483648; 
int encoder_min =-2147483648; 

//Thresholds to determine if wrap around has occurred 
int encoder_low_wrap; 
int encoder_high_wrap; 

// Update the left encoder count
void update_encoder_left(int count)
{	
	if(count < encoder_low_wrap && left_prev_encoder > encoder_high_wrap)
	{
		//we wrapped around the high side, we're moving in the postive 
		//direction so this diff needs to be positive 
		int prev_diff_to_high=encoder_max - left_prev_encoder; 
		int diff_from_low= count-encoder_min; 
		
		left_encoder_diff = prev_diff_to_high + diff_from_low; 
			
	}
            
        
	else if(count > encoder_high_wrap && left_prev_encoder < encoder_low_wrap)
	{
		//we wrapped around the low side, we're moving in the negative
		//direction so this diff needs to be negative 
		int prev_diff_to_low=encoder_min - left_prev_encoder;  
		int diff_from_high= count- encoder_max; 
		
		left_encoder_diff = prev_diff_to_low + diff_from_high; 
		
	}
	
	//Otherwise just take the simple difference in counts
	
	else 
	{
		left_encoder_diff = count- left_prev_encoder; 
	}
	
	left_prev_encoder = count; 
		
}

// Update the right encoder count
void update_encoder_right(int count)
{
	
    if(count < encoder_low_wrap && left_prev_encoder > encoder_high_wrap)
	{
		//we wrapped around the high side, we're moving in the postive 
		//direction so this diff needs to be positive 
		int prev_diff_to_high=encoder_max - right_prev_encoder; 
		int diff_from_low= count-encoder_min; 
		
		right_encoder_diff = prev_diff_to_high + diff_from_low; 
			
	}
            
        
	else if(count > encoder_high_wrap && left_prev_encoder < encoder_low_wrap)
	{
		//we wrapped around the low side, we're moving in the negative
		//direction so this diff needs to be negative 
		int prev_diff_to_low=encoder_min - right_prev_encoder ; 
		int diff_from_high= count- encoder_max; 
		
		right_encoder_diff = prev_diff_to_low + diff_from_high; 
		
	}
	
	//Otherwise just take the simple difference in counts
	
	else 
	{
		right_encoder_diff = count-right_prev_encoder; 
	}
	
	//Update our previous encoder count 
	right_prev_encoder = count; 
}


void leftEncoderCallback(const std_msgs::Int32& message_holder)
{
    left_encoder_count = message_holder.data; 
    //ROS_INFO("Left encoder count received by odometry node %i : ", left_encoder_count); 
}

/*!
 * \brief callback when the right encoder count changes
 * \param ptr encoder parameters
 */
void rightEncoderCallback(const std_msgs::Int32& message_holder)
{
    right_encoder_count = message_holder.data;  
    //ROS_INFO("Right encoder count received by odometry node %i: ", right_encoder_count); 
    
}

void odomResetCallback(const std_msgs::Bool& message_holder)
{
    g_odom_reset = message_holder.data;  
    
}


bool subscribe_to_encoders()
{
	bool success = true;
	ros::NodeHandle n;
	ros::NodeHandle nh("~");
	std::string left_topic_path = "phidgets/";
	
	nh.getParam("left_topic_path", left_topic_path);
	
	std::string right_topic_path = "phidgets/";
	nh.getParam("right_topic_path", right_topic_path);
	

	// subscribe to the topic
	ROS_INFO("Left encoder: %s", left_topic_path.c_str());
	left_encoder_sub =
		n.subscribe(left_topic_path, 1,
					leftEncoderCallback);
	
	ROS_INFO("Right encoder: %s",
			 right_topic_path.c_str());
	right_encoder_sub =
		n.subscribe(right_topic_path, 1,
					rightEncoderCallback);
	
	
	return(success);
}

/*!
 * \brief updates the robot velocities
 * \param dt time elapsed in seconds
 */
void update_position_orientation_velocity(
					   double dt)
{
	update_encoder_right(right_encoder_count);
	update_encoder_left(left_encoder_count);
	
	double local_x, local_y; 
	
    if (dt > 0) {
        
        double cos_current = cos(theta);
        double sin_current = sin(theta);

        int dist_left_counts = left_encoder_diff; 
        int dist_right_counts = right_encoder_diff; 

       
		// convert from counts to standard units
		double dist_left_mm =
			(double)dist_left_counts /
			left_encoder_counts_per_mm;
		double dist_right_mm =
			(double)dist_right_counts /
			right_encoder_counts_per_mm;
			
		const double mm_to_m = 1.0 / 1000; 
				double local_theta = (dist_right_mm - dist_left_mm) / wheelbase_mm ; 

        theta += local_theta; 
		D = (dist_right_mm + dist_left_mm) * 0.5*mm_to_m;  

		
		// calculate distance traveled in x and y
		local_x = cos(theta) * D; 
		local_y = sin(theta) * D; 
		// calculate the final position of the robot
		x += local_x; 
		y += local_y; 
			
	
		vD = D / dt; 
		vtheta = local_theta / dt; 
		vx=local_x / dt; 
		vy=local_y / dt; 
			
    }
}

int main(int argc, char** argv)
{
    

	ros::init(argc, argv, "phidgets_odometry");

	ros::NodeHandle n;
	ros::NodeHandle nh("~");

	std::string name = "odom";
	nh.getParam("name", name);

	nh.getParam("rotation", rotation_offset);
		
	encoder_low_wrap= 0.3 * (encoder_max - encoder_min) + encoder_min; 
	encoder_high_wrap= 0.7 * (encoder_max - encoder_min) + encoder_min; 

	ros::Publisher odom_pub =
		n.advertise<nav_msgs::Odometry>(name, 50);
		
	odom_reset_pub = n.advertise<std_msgs::Bool>("odom_reset", 1);	
	
	ros::Subscriber odom_reset_sub= 
	n.subscribe("odom_reset",1,odomResetCallback); 
		
	tf::TransformBroadcaster odom_broadcaster;

	int direction = -1;
	nh.getParam("encoderdirectionleft", direction);
	if (direction!=-1) encoder_direction_left = 1;
	direction=1;
	nh.getParam("encoderdirectionright",
				encoder_direction_right);
	if (direction!=1) encoder_direction_right = -1;

	// connect to the encoders
	bool subscribed_to_encoders = false;
	
	subscribed_to_encoders = subscribe_to_encoders();
	

	if (subscribed_to_encoders) {

		std::string base_link = "base_link";
		nh.getParam("base_link", base_link);
		std::string frame_id = "odom";
		nh.getParam("frame_id", frame_id);
		left_encoder_counts_per_mm = 0;
		right_encoder_counts_per_mm = 0;
		nh.getParam("countspermmleft",
					left_encoder_counts_per_mm);
		if (left_encoder_counts_per_mm < 1) {
			left_encoder_counts_per_mm = 1000;
		}
		nh.getParam("countspermmright",
					right_encoder_counts_per_mm);
		if (right_encoder_counts_per_mm < 1) {
			right_encoder_counts_per_mm = 1000;
		}
		wheelbase_mm = 0;
		nh.getParam("wheelbase", wheelbase_mm);
		if (wheelbase_mm < 1) wheelbase_mm = 400;

		nh.getParam("verbose", verbose);
		nh.setParam("verbose", false);

		double frequency = 100;
		nh.getParam("frequency", frequency);

		ros::Time current_time, last_time;
		current_time = ros::Time::now();
		last_time = ros::Time::now();

		initialised = true;

		ros::Rate update_rate(frequency);
		while(ros::ok()){
			// reset the pose
			
			if (g_odom_reset) {
				x = 0;
				y = 0;
				theta = 0;
				vx = 0;
				vy = 0;
				vtheta = 0;
				
				std_msgs::Bool reset; 
				reset.data = false; 
				
				//could update g_odom_reset here but might as well wait
				//until it is updated by the topic. 
				
				odom_reset_pub.publish(reset);
				start_encoder_count_left = 0;
				start_encoder_count_right = 0;
			}

			current_time = ros::Time::now();
			double dt = (current_time - last_time).toSec();

			// update the velocity estimate based upon
			// encoder values
			update_position_orientation_velocity(dt);

			// since all odometry is 6DOF we'll need a
			// quaternion created from yaw
			geometry_msgs::Quaternion odom_quat =
				tf::createQuaternionMsgFromYaw(theta);

			// first, we'll publish the transform over tf
			geometry_msgs::TransformStamped odom_trans;
			odom_trans.header.stamp = current_time;
			odom_trans.header.frame_id = frame_id;
			odom_trans.child_frame_id = base_link;

			odom_trans.transform.translation.x = x;
			odom_trans.transform.translation.y = y;
			odom_trans.transform.translation.z = 0.0;
			odom_trans.transform.rotation = odom_quat;

			// send the transform
			odom_broadcaster.sendTransform(odom_trans);

			// next, we'll publish the odometry
			//  message over ROS
			nav_msgs::Odometry odom;
			odom.header.stamp = current_time;
			odom.header.frame_id = frame_id;

			// set the position
			odom.pose.pose.position.x = x;
			odom.pose.pose.position.y = y;
			odom.pose.pose.position.z = 0.0;
			odom.pose.pose.orientation = odom_quat;

			// set the velocity
			odom.child_frame_id = base_link;
			odom.twist.twist.linear.x = vD;
			odom.twist.twist.linear.y = 0.0;
			odom.twist.twist.angular.z = vtheta;

			// publish the message
			odom_pub.publish(odom);

			last_time = current_time;
			ros::spinOnce();
			update_rate.sleep();
		}
        
    }
}

