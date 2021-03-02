//------------------------------INCLUDE SECTION------------------------------//
#include "ros/ros.h"
#include "dc_motor_controller/Commands.h"
#include "dc_motor_controller/DCMotorControl.h"
#include "sensor_msgs/Joy.h"


//------------------------GLOBAL VARIABLE DECLARATION------------------------//
ros::ServiceClient leadingDrive;
ros::ServiceClient trailingDrive;
ros::ServiceClient leadingLifter;
ros::ServiceClient trailingLifter;


//----------------------------FUNCTION PROTOTYPES----------------------------//
void joystickCallback(const sensor_msgs::Joy::ConstPtr& msg);


//-------------------------MAIN FUNTION DECLARATION--------------------------//
int main(int argc, char **argv)
{
	//Initiate ROS system and get a node handle
	ros::init(argc, argv, "robot_joystick_controller");
	ros::NodeHandle n;
  
	//Create a subscriber for the joystick
	ros::Subscriber sub = n.subscribe("joy", 1000, joystickCallback);

	//Create service clients for all controlling motors
	leadingDrive = n.serviceClient<dc_motor_controller::DCMotorControl>("/leading_motor/motor_control");
	trailingDrive = n.serviceClient<dc_motor_controller::DCMotorControl>("/trailing_motor/motor_control");
	leadingLifter = n.serviceClient<dc_motor_controller::DCMotorControl>("/leading_lifter/motor_control");
	trailingLifter = n.serviceClient<dc_motor_controller::DCMotorControl>("/trailing_lifter/motor_control");

	//Update when new joystick data is given
	ros::spin();
}



//----------------------SUPPORTING FUNTION DECLARATION----------------------//
void joystickCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
	//Calculate the common-mode speed of the tracks
	int32_t trailingDriveSpeed = -msg->axes[0]*2500;
	int32_t leadingDriveSpeed = trailingDriveSpeed;

	//Calculate differential speed of the tracks
	trailingDriveSpeed += msg->axes[2]*1250;
	leadingDriveSpeed += msg->axes[2]*1250;

	//Calculate the tilt power of the lifters
	int32_t trailingLifterPower = msg->axes[1]*255;
	int32_t leadingLifterPower = -msg->axes[1]*255;
	
	if(msg->buttons[11])
	{
		trailingLifterPower = 200;
	}
	
	if(msg->buttons[10])
	{
		trailingLifterPower = -200;
	}
	
	if(msg->buttons[7])
	{
		leadingLifterPower = 200;
	}
	
	if(msg->buttons[6])
	{
		leadingLifterPower = -200;
	}

	//Send control signals to the motor controllers
	dc_motor_controller::DCMotorControl serviceCaller;

	  //Leading drive motor
	serviceCaller.request.command = COMMAND_CONSTANT_SPEED_ENCODER;
	serviceCaller.request.parameter = leadingDriveSpeed;
	if(!leadingDrive.call(serviceCaller))
	{
		ROS_WARN("%s: Error sending command to leading drive node", ros::this_node::getName().c_str());
	}

	  //Trailing drive motor
	serviceCaller.request.command = COMMAND_CONSTANT_SPEED_ENCODER;
	serviceCaller.request.parameter = trailingDriveSpeed;
	if(!trailingDrive.call(serviceCaller))
	{
		ROS_WARN("%s: Error sending command to trailing drive node", ros::this_node::getName().c_str());
	}

	  //Leading lifter motor
	serviceCaller.request.command = COMMAND_CONSTANT_VOLTAGE;
	serviceCaller.request.parameter = leadingLifterPower;
	if(!leadingLifter.call(serviceCaller))
	{
		ROS_WARN("%s: Error sending command to leading lifter node", ros::this_node::getName().c_str());
	}

	  //Trailing lifter motor
	serviceCaller.request.command = COMMAND_CONSTANT_VOLTAGE;
	serviceCaller.request.parameter = trailingLifterPower;
	if(!trailingLifter.call(serviceCaller))
	{
		ROS_WARN("%s: Error sending command to trailing lifter node", ros::this_node::getName().c_str());
	}
}
