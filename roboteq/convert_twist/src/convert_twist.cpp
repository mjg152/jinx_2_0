#include <ros/ros.h> 
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <roboteq_msgs/Command.h>
#include <roboteq_msgs/Feedback.h>
#include <roboteq_msgs/Status.h>
#include <sensor_msgs/JointState.h>

//this node takes a geometry msgs twist in m/s and rad/s
//and publishes roboteq_msgs to the 2 channels
ros::Publisher ch1, ch2;

float MAX_SPEED_RPM = 1000; //this should be set the same as what is in the roboteq mbs script 
float WHEEL_RADIUS = 0.1651;
float GEAR_RATIO = 18.33;
float LEFT_OVER_RIGHT = 0.997;
float g_pos1, g_pos2, g_vel1, g_vel2;

float LOOP_RATE = 50.0;
float MAX_LIN_ACEL = 1.0;
float MAX_ANG_ACEL = 1.0;

float DRIVING_DIRECTION = -1.0;

geometry_msgs::Twist g_last_sent_cmd;
geometry_msgs::Twist g_last_received_cmd;

bool g_dig1_status; 


void twistCallback(const geometry_msgs::Twist& message_holder) 
{	
	g_last_received_cmd = message_holder;

}

void statusCallback(const roboteq_msgs::Status& message_holder) 
{	
	g_dig1_status = message_holder.DIN_1;	
	ROS_INFO("We're in the twist callback and received this for DIN_1 %u", g_dig1_status); 
	
}

int main(int argc, char **argv) 
{ 


	ros::init(argc,argv,"convert_twist");
	ros::NodeHandle nh_;
	ros::Subscriber cmd_converter= nh_.subscribe("cmd_vel",1,twistCallback);
	ros::Subscriber status= nh_.subscribe("roboteq_driver/status",1,statusCallback);
	ch1 = nh_.advertise<roboteq_msgs::Command>("ch1/cmd", 1);
	ch2 = nh_.advertise<roboteq_msgs::Command>("ch2/cmd", 1);
	g_pos1 = 0;
	g_pos2 = 0;
	g_vel1 = 0;
	g_vel2 = 0;

	g_last_sent_cmd.linear.x = 0;
	g_last_sent_cmd.angular.z = 0;

	ros::Rate looprate(LOOP_RATE);
	while (ros::ok()){
	
		looprate.sleep();
		ros::spinOnce();

		float v1,v2;
		
		//Set up v1 and v2 and correct for + - sign convention 
		
		v1 = g_last_received_cmd.linear.x*DRIVING_DIRECTION;
		v2 = g_last_received_cmd.linear.x*DRIVING_DIRECTION;

		//Now add some differential speed to accomplish the turn based on
		//Our Z commands. Convert to meter's per second on the angular 
		//Velocity before adding. 
		
		v2 += (g_last_received_cmd.angular.z)*WHEEL_RADIUS;
		v1 -= (g_last_received_cmd.angular.z)*WHEEL_RADIUS;

		//Our final command is generated as a ratio of our maximum allowed RPM, so we need 
		//to back out our base_radius so convert from m/s to rotations per second
		
		v1/= 2*3.14159*WHEEL_RADIUS;
		v2/= 2*3.14159*WHEEL_RADIUS; 
		
		//Now convert from rotations per second to RPM
		v1*= 60; 
		v2*= 60; 
		
		//Really we're commanding a wheel speed, so we also need to account
		//For our gearbox ratio:
		
		v1*= GEAR_RATIO;
		v2*= GEAR_RATIO;
		
		//Saturate against the maximum allowed speed. 

		v1 = std::max(-1.0f*MAX_SPEED_RPM,std::min(v1,MAX_SPEED_RPM));
		v2 = std::max(-1.0f*MAX_SPEED_RPM,std::min(v2,MAX_SPEED_RPM));
		
		//Now we need to convert v1 and v2 back to the units the 
		//roboteq driver expects to see, rad/s
		
		//v1*= (2*3.14159)/60;
		//v2*= (2*3.14159)/60;
		

		roboteq_msgs::Command cmd1, cmd2;
		cmd1.mode = 0;
		cmd2.mode = 0;
		cmd1.setpoint = v1;
		cmd2.setpoint = v2;

		ch1.publish(cmd1);
		ch2.publish(cmd2);

	}
	return 0;
}
