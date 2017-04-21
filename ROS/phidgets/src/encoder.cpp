/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  ROS driver for Phidgets high speed encoder
 *  Copyright (c) 2016
 * 	
 * 	This driver borrows heavily from the example source code provided by
 *  Phidgets at the following address: 	
 * 
 *  http://www.phidgets.com/downloads/examples/phidget21-c- ...
 *  examples_2.1.8.20151217.tar.gz
 * 
 *  Additionally software is borrowed Bob Mottram's open source 
 *  Phidgets library which is summarized here: 
 * 
 * 	http://wiki.ros.org/phidgets
 * 
 *  Remaining content by Mike Gallagher
 *  e-mail: mjg152@case.edu
 * 
 *  All rights reserved.
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

#include <ros/ros.h>
#include <stdio.h>
#include <std_msgs/Int32.h>
#include <libphidgets/phidget21.h>
#include "phidgets/encoder_params.h"


// Handle
CPhidgetEncoderHandle phid;
CPhidgetManagerHandle m_phid;

// encoder state publishers
ros::Publisher encoder_0;
ros::Publisher encoder_1;

bool initialised = false;

int CCONV AttachHandler(CPhidgetHandle ENC, void *userptr)
{
        int serialNo;
        CPhidget_DeviceID deviceID;
        int i, inputcount;

        CPhidget_getSerialNumber(ENC, &serialNo);

        //Retrieve the device ID and number of encoders so that we can set the enables if needed
        CPhidget_getDeviceID(ENC, &deviceID);
        CPhidgetEncoder_getEncoderCount((CPhidgetEncoderHandle)ENC, &inputcount);
        ROS_INFO("Encoder %10d attached! \n", serialNo);

        //the 1047 requires enabling of the encoder inputs, so enable them if this is a 1047    
        if (deviceID == PHIDID_ENCODER_HS_4ENCODER_4INPUT)
        {
                ROS_INFO("Encoder requires Enable. Enabling inputs....\n");
                for (i = 0 ; i < inputcount ; i++)
                        CPhidgetEncoder_setEnabled((CPhidgetEncoderHandle)ENC, i, 1);
        }
        return 0;
}

int CCONV DetachHandler(CPhidgetHandle ENC, void *userptr)
{
	int serialNo;
	CPhidget_getSerialNumber(ENC, &serialNo);
	ROS_INFO("Encoder %10d detached! \n", serialNo);

	return 0;
}

int CCONV ErrorHandler(CPhidgetHandle ENC, void *userptr, int ErrorCode, const char *Description)
{
	ROS_INFO("Error handled. %d - %s \n", ErrorCode, Description);

	return 0;
}

int CCONV InputChangeHandler(CPhidgetEncoderHandle ENC, void *usrptr, int Index, int State)
{
	ROS_INFO("Input #%i - State: %i \n", Index, State);

	return 0;
}

int CCONV PositionChangeHandler(CPhidgetEncoderHandle ENC, void *usrptr, int Index, int Time, int RelativePosition)
{
	int Position;
	CPhidgetEncoder_getPosition(ENC, Index, &Position);
	std_msgs::Int32 pub_position; 
		
	if (Index == 0 )
	{
		phidgets::encoder_params e;
		e.index = Index;
		e.count = Position;
		e.count_change = RelativePosition;
		e.time = Time;
		
		
		pub_position.data=-Position; 
		
		encoder_0.publish(pub_position);
	}
    
    else if (Index == 1)
	{
		phidgets::encoder_params e;
		e.index = Index;
		e.count = Position;
		e.count_change = RelativePosition;
		e.time = Time;
		
		pub_position.data=Position; 
		
		encoder_1.publish(pub_position);
	}
    
    ROS_INFO("Encoder %d Count %d", Index, Position);
	
	return 0;
}

//Display the properties of the attached phidget to the screen.  We will be displaying the name, serial number and version of the attached device.
//Will also display the number of inputs and encoders on this device
int display_properties(CPhidgetEncoderHandle phid)
{
	int serialNo, version, num_inputs, num_encoders;
	const char* ptr;

	CPhidget_getDeviceType((CPhidgetHandle)phid, &ptr);
	CPhidget_getSerialNumber((CPhidgetHandle)phid, &serialNo);
	CPhidget_getDeviceVersion((CPhidgetHandle)phid, &version);

	CPhidgetEncoder_getInputCount(phid, &num_inputs);
	CPhidgetEncoder_getEncoderCount(phid, &num_encoders);

	ROS_INFO("%s\n", ptr);
	ROS_INFO("Serial Number: %10d\nVersion: %8d\n", serialNo, version);
	ROS_INFO("Num Encoders: %d\nNum Inputs: %d\n", num_encoders, num_inputs);

	return 0;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "phidgets_high_speed_encoder");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");
    encoder_0= n.advertise<std_msgs::Int32>("phidgets/encoder0", 1);
    encoder_1 = n.advertise<std_msgs::Int32>("phidgets/encoder1", 1);

	//Declare an encoder handle
	CPhidgetEncoderHandle encoder = 0;
	
	while(ros::ok())
	{
		int result;
		const char *err;

		//create the encoder object
		CPhidgetEncoder_create(&encoder);

		//Set the handlers to be run when the device is plugged in or opened from software, unplugged or closed from software, or generates an error.
		CPhidget_set_OnAttach_Handler((CPhidgetHandle)encoder, AttachHandler, NULL);
		CPhidget_set_OnDetach_Handler((CPhidgetHandle)encoder, DetachHandler, NULL);
		CPhidget_set_OnError_Handler((CPhidgetHandle)encoder, ErrorHandler, NULL);

		//Registers a callback that will run if an input changes.
		//Requires the handle for the Phidget, the function that will be called, and an arbitrary pointer that will be supplied to the callback function (may be NULL).
		CPhidgetEncoder_set_OnInputChange_Handler(encoder, InputChangeHandler, NULL);

		//Registers a callback that will run if the encoder changes.
		//Requires the handle for the Encoder, the function that will be called, and an arbitrary pointer that will be supplied to the callback function (may be NULL).
		CPhidgetEncoder_set_OnPositionChange_Handler (encoder, PositionChangeHandler, NULL);

		CPhidget_open((CPhidgetHandle)encoder, -1);

		//get the program to wait for an encoder device to be attached
		printf("Waiting for encoder to be attached....");
		if((result = CPhidget_waitForAttachment((CPhidgetHandle)encoder, 10000)))
		{
			CPhidget_getErrorDescription(result, &err);
			printf("Problem waiting for attachment: %s\n", err);
			return 0;
		}

		//Display the properties of the attached encoder device
		display_properties(encoder);
		
		//Now let's just spin and let the callbacks get handled 
		ros::spin(); 
	}
    
    CPhidget_close((CPhidgetHandle)encoder);
	CPhidget_delete((CPhidgetHandle)encoder);
    
    return 0;
}

