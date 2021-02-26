//------------------------------INCLUDE SECTION------------------------------//
#include "ros/ros.h"
#include "robobus/SerialRead.h"
#include "robobus/SerialWrite.h"
#include "dc_motor_controller/Commands.h"
#include "dc_motor_controller/DCMotorControl.h"



//------------------------GLOBAL VARIABLE DECLARATION------------------------//
uint8_t deviceBusy = 0;

uint16_t returnedBusValue = 0;
uint8_t newBusValue = 0;

int address = 127;
int addressType = 0;
ros::ServiceClient serialWriteClient;



//---------------------------FUNCTION PROTOTYPES----------------------------//
void serialReadCallback(const robobus::SerialRead::ConstPtr &msg);
bool controlMotor(dc_motor_controller::DCMotorControl::Request  &req,
		  dc_motor_controller::DCMotorControl::Response &res);


//-------------------------MAIN FUNTION DECLARATION-------------------------//
int main(int argc, char **argv)
{
	//Initiate ROS system and get a node handle
	ros::init(argc, argv, "dc_motor_controller");
	ros::NodeHandle n;
  
	//Obtain and save the address for this particular node instance
	ros::param::get(ros::this_node::getName() + "address", address);
        ros::param::get(ros::this_node::getName() + "is_group_address", addressType);

	//Create a service client for the robobus node
	serialWriteClient = n.serviceClient<robobus::SerialWrite>("/serial_write");

	//Obtain and write Kp, Ki, Kd, and outputDivisior values to the controller
	  //Obtain the Kp, Ki, Kd, and divisor values
	int Kp, Ki, Kd, outputDivisor;
	if(!ros::param::get("/" + ros::this_node::getName() + "/kp", Kp)){
	
		ROS_WARN("DCMotorDriver: Kp parameter not found, defaulting to 0");
		Kp = 0;
	}

	if(!ros::param::get("/" + ros::this_node::getName() + "/ki", Ki))
	{
		ROS_WARN("DCMotorDriver: Ki parameter not found, defaulting to 0");
		Ki = 0;
	}

	if(!ros::param::get("/" + ros::this_node::getName() + "/kd", Kd))
	{
		ROS_WARN("DCMotorDriver: Kd parameter not found, defaulting to 0");
		Kd = 0;
	}

	if(!ros::param::get("/" + ros::this_node::getName() + "/output_divisor", outputDivisor))
	{
		ROS_WARN("DCMotorDriver: outputDivisor parameter not found, defaulting to 1");
		outputDivisor = 1;
	}

	  //Write Kp, Ki, Kd, and divisor values to motor controller
	robobus::SerialWrite parameterRequest;
	parameterRequest.request.address_type = addressType;
	parameterRequest.request.address = address;
	parameterRequest.request.command = COMMAND_EDIT_KP;
	parameterRequest.request.parameter = Kp;

	while(!serialWriteClient.call(parameterRequest))
	{
		ROS_ERROR("DCMotorDriver: Kp transmit request failed. Look at robobus logs for details");
	}

	parameterRequest.request.command = COMMAND_EDIT_KI;
	parameterRequest.request.parameter = Ki;

	while(!serialWriteClient.call(parameterRequest))
	{
		ROS_ERROR("DCMotorDriver: Ki transmit request failed. Look at robobus logs for details");
	}

	parameterRequest.request.command = COMMAND_EDIT_KD;
	parameterRequest.request.parameter = Kd;

	while(!serialWriteClient.call(parameterRequest))
	{
		ROS_ERROR("DCMotorDriver: Kd transmit request failed. Look at robobus logs for details");
	}

	parameterRequest.request.command = COMMAND_EDIT_OUTPUT_DIVISOR;
	parameterRequest.request.parameter = outputDivisor;

	while(!serialWriteClient.call(parameterRequest))
	{
		ROS_ERROR("DCMotorDriver: Output divisor transmit request failed. Look at robobus logs for details");
	}
        
        //Start the dc motor control service and subscribe to the robobus serial read node
	ros::ServiceServer controlService = n.advertiseService(
	"/" + ros::this_node::getName() + "/motor_control", controlMotor);
	
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



bool controlMotor(dc_motor_controller::DCMotorControl::Request  &req,
		  dc_motor_controller::DCMotorControl::Response &res)
{
	//Only send a new command if the controller isn't busy
	if(!deviceBusy)
	{
		uint8_t requiresCommandReception = 0;

		//Construct robobus request
		robobus::SerialWrite motorRequest;
		motorRequest.request.address_type = addressType;
		motorRequest.request.address = address;
		motorRequest.request.command = req.command;

		//Find if command requires the reception of data
		if(req.command == COMMAND_GET_CURRENT ||
		   req.command == COMMAND_GET_SPEED   ||
		   req.command == COMMAND_GET_POSITION )
		{
			requiresCommandReception = 1;
		}

		//Find proper parameter value depending on the command sent
		switch(req.command)
		{
			//If command is COMMAND_CONSTANT_VOLTAGE, saturate the input to 255
			case COMMAND_CONSTANT_VOLTAGE:
				if(req.parameter > 255)
				{
					motorRequest.request.parameter = 255;
				}
				else if(req.parameter < -255)
				{
					motorRequest.request.parameter = -255;
				}
				else
				{
					motorRequest.request.parameter = req.parameter;
				}
			break;

			//If command is COMMAND_CONSTANT_CURRENT, convert the input to mA
			case COMMAND_CONSTANT_CURRENT:
			{
				//Refer to DC Motor Controller V3 Datasheet section 5.3 for calculation
				int16_t calculatedCurrent = req.parameter*2.03558788;
				motorRequest.request.parameter = calculatedCurrent;
			}
			break;

			//If command is COMMAND_CONSTANT_SPEED_ENCODER, convert input to ticks/second
			case COMMAND_CONSTANT_SPEED_ENCODER:
			{
				//Refer to DC Motor Controller V3 Datasheet section 6.4 for calculation
				int16_t calculatedSpeed = req.parameter/50000;
				motorRequest.request.parameter = calculatedSpeed;
			}
			break;

			//Otherwise, directly feed parameter to serial output
			defult:
				motorRequest.request.parameter = req.command;
			break;
		}

		//Send the request
		newBusValue = 0;
		if(!serialWriteClient.call(motorRequest))
		{
			ROS_WARN("DCMotorDriver: Command transmission request failed. Check robobus node for details");
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
				res.value = returnedBusValue;
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
