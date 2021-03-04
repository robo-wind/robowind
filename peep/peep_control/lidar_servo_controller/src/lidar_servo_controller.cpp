//------------------------------INCLUDE SECTION------------------------------//
#include "ros/ros.h"
#include "robobus/SerialRead.h"
#include "robobus/SerialWrite.h"
#include "lidar_servo_controller/Commands.h"
#include "lidar_servo_controller/LidarServoControl.h"



//------------------------GLOBAL VARIABLE DECLARATION------------------------//
uint8_t deviceBusy = 0;

uint16_t returnedBusValue = 0;
uint8_t newBusValue = 0;

int address = 127;
int addressType = 0;
ros::ServiceClient serialWriteClient;



//----------------------------FUNCTION PROTOTYPES---------------------------//
void serialReadCallback(const robobus::SerialRead::ConstPtr &msg);
bool controlLidarServo(lidar_servo_controller::LidarServoControl::Request  &req,
		  lidar_servo_controller::LidarServoControl::Response &res);



//-------------------------MAIN FUNTION DECLARATION-------------------------//
int main(int argc, char **argv)
{
	//Initiate ROS system and get a node handle
	ros::init(argc, argv, "lidar_servo_controller");
	ros::NodeHandle n;

	//Obtain and save the address for this particular node instance
	ros::param::get(ros::this_node::getName() + "/address", address);
	ros::param::get(ros::this_node::getName() + "/is_group_address", addressType);

	//Create a service client for the robobus node
	serialWriteClient = n.serviceClient<robobus::SerialWrite>("/serial_write");

	//Start the lidar servo control service and subscribe to the robobus serial read node
	ros::ServiceServer controlService = n.advertiseService(
			"/" + ros::this_node::getName() + "/lidar_servo_control", controlLidarServo);
	
	ros::Subscriber serialReadSubscriber = n.subscribe("/serial_read", 1000, serialReadCallback);

	//Execute until ROS termination
	ros::spin();

	return 0;
}



//----------------------SUPPORTING FUNTION DECLARATION----------------------//
void serialReadCallback(const robobus::SerialRead::ConstPtr &msg)
{
	//If we receive a response that is addressed to this controller, save the message
	if(msg->address == address)
	{
		newBusValue = 1;
		returnedBusValue = msg->data;
	}
}



bool controlLidarServo(lidar_servo_controller::LidarServoControl::Request  &req,
		  lidar_servo_controller::LidarServoControl::Response &res)
{
	//Only send a new command if the controller isn't busy
	if(!deviceBusy)
	{
		uint8_t requiresCommandReception = 0;

		//Construct robobus request
		robobus::SerialWrite lidarServoRequest;
		lidarServoRequest.request.address_type = addressType;
		lidarServoRequest.request.address = address;
		lidarServoRequest.request.command = req.command;

		//Find if command requires the reception of data
		if(req.command != COMMAND_SET_SERVO_YAW   &&
		   req.command != COMMAND_SET_SERVO_PITCH )
		{
			requiresCommandReception = 1;
		}

		//Find proper parameter value depending on the command sent
		switch(req.command)
		{
			case COMMAND_SET_SERVO_YAW:
				//TODO: Find angle to servo delay value and apply here
				lidarServoRequest.request.parameter = req.command;
			break;
			
			case COMMAND_SET_SERVO_PITCH:
				//TODO: Find angle to servo delay value and apply here
				lidarServoRequest.request.parameter = req.command;
			break;
		
			//Otherwise, directly feed parameter to serial output
			default:
				lidarServoRequest.request.parameter = req.command;
			break;
		}

		//Send the request
		newBusValue = 0;
		if(!serialWriteClient.call(lidarServoRequest))
		{
			ROS_WARN("%s: Command transmission request failed. Check robobus node for details",
					ros::this_node::getName().c_str());
			return false;
		}

		//If needed, wait for a response from the controller
		while(requiresCommandReception == 1)
		{
			if(!newBusValue)
			{
				deviceBusy = 1;
				ros::spinOnce();
			}
			else
			{
				deviceBusy = 0;
				newBusValue = 0;
				requiresCommandReception = 2;
			}
		}

		//Return the value obtained from the bus
		if(requiresCommandReception)
		{
			res.value = returnedBusValue;
		}else
		{
			res.value = 0;
		}
	}

  	return true;
}
