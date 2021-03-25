//------------------------------INCLUDE SECTION------------------------------//
#include "ros/ros.h"
#include "robobus/SerialWrite.h"
#include "ac_motor_controller/Commands.h"
#include "ac_motor_controller/ACMotorControl.h"



//------------------------GLOBAL VARIABLE DECLARATION------------------------//
int address = 127;
int addressType = 0;
ros::ServiceClient serialWriteClient;


bool controlMotor(ac_motor_controller::ACMotorControl::Request  &req,
		  ac_motor_controller::ACMotorControl::Response &res);


//-------------------------MAIN FUNTION DECLARATION-------------------------//
int main(int argc, char **argv)
{
	//Initiate ROS system and get a node handle
	ros::init(argc, argv, "ac_motor_controller");
	ros::NodeHandle n;
  
	//Obtain and save the address for this particular node instance
	ros::param::get(ros::this_node::getName() + "address", address);
        ros::param::get(ros::this_node::getName() + "is_group_address", addressType);

	//Create a service client for the robobus node
	serialWriteClient = n.serviceClient<robobus::SerialWrite>("/serial_write");
        
        //Start the dc motor control service and subscribe to the robobus serial read node
	ros::ServiceServer controlService = n.advertiseService(
	"/" + ros::this_node::getName() + "/motor_control", controlMotor);	

	//Execute until ROS termination
	ros::spin();

	return 0;
}



//----------------------SUPPORTING FUNTION DECLARATION----------------------//
bool controlMotor(ac_motor_controller::ACMotorControl::Request  &req,
		  ac_motor_controller::ACMotorControl::Response &res)
{
	//Construct robobus request
	robobus::SerialWrite motorRequest;
	motorRequest.request.address_type = addressType;
	motorRequest.request.address = address;
	motorRequest.request.command = req.command;

	//Send the request
	if(!serialWriteClient.call(motorRequest))
	{
		ROS_WARN("%s: Command transmission request failed. Check robobus node for details", ros::this_node::getName().c_str());
		return false;
	}

  	return true;
}
