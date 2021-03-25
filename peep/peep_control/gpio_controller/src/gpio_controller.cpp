//------------------------------INCLUDE SECTION------------------------------//
#include "ros/ros.h"
#include "robobus/SerialRead.h"
#include "robobus/SerialWrite.h"
#include "gpio_controller/Commands.h"
#include "gpio_controller/GPIOControl.h"



//------------------------GLOBAL VARIABLE DECLARATION------------------------//
uint8_t deviceBusy = 0;

uint16_t returnedBusValue = 0;
uint8_t newBusValue = 0;

int address = 127;
int addressType = 0;
ros::ServiceClient serialWriteClient;



void serialReadCallback(const robobus::SerialRead::ConstPtr &msg);
bool controlGPIO(gpio_controller::GPIOControl::Request  &req,
		  gpio_controller::GPIOControl::Response &res);


//-------------------------MAIN FUNTION DECLARATION-------------------------//
int main(int argc, char **argv)
{
	//Initiate ROS system and get a node handle
	ros::init(argc, argv, "gpio_controller");
	ros::NodeHandle n;
  
	//Obtain and save the address for this particular node instance
	ros::param::get(ros::this_node::getName() + "/address", address);
        ros::param::get(ros::this_node::getName() + "/is_group_address", addressType);

	//Create a service client for the robobus node
	serialWriteClient = n.serviceClient<robobus::SerialWrite>("/serial_write");

        //Start the dc motor control service and subscribe to the robobus serial read node
	ros::ServiceServer controlService = n.advertiseService(
	"/" + ros::this_node::getName() + "/gpio_control", controlGPIO);
	
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



bool controlGPIO(gpio_controller::GPIOControl::Request  &req,
		  gpio_controller::GPIOControl::Response &res)
{
	//Only send a new command if the controller isn't busy
	if(!deviceBusy)
	{
		uint8_t requiresCommandReception = 0;

		//Construct robobus request
		robobus::SerialWrite gpioRequest;
		gpioRequest.request.address_type = addressType;
		gpioRequest.request.address = address;
		gpioRequest.request.command = req.command;
		
		for(int index = 0; index < 6; index++)
		{
			gpioRequest.request.parameter |= req.parameter_bits[index] << index;
		
		}

		//Find if command requires the reception of data
		if(req.command == COMMAND_GET_INPUT_DATA)
		{
			requiresCommandReception = 1;
		}

		//Send the request
		newBusValue = 0;
		if(!serialWriteClient.call(gpioRequest))
		{
			ROS_WARN("GPIOController: Command transmission request failed. Check robobus node for details");
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
