//----------INCLUDES----------//
#include "ros/ros.h"
#include "robobus/SerialWrite.h"
#include "robobus/SerialRead.h"

#include <sstream>
#include <string>
#include <stdio.h>

#include <sys/ioctl.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>


#define COMMAND_TRANSMIT_BUFFER_SIZE  100
#define COMMAND_RECEIVE_BUFFER_SIZE  100



//----------GLOBAL VARIABLE DECLARATIONS----------//
robobus::SerialWrite::Request requestBuffer[COMMAND_RECEIVE_BUFFER_SIZE];
int currentBufferSize = 0;


//----------FUNCTION PROTOTYPES----------//
void updateWriteOperations(int serialPort);
void updateReadOperations(int serialPort, ros::Publisher readTopic);
int initSerialPort();
bool serialWrite(robobus::SerialWrite::Request &request,
                 robobus::SerialWrite::Response &response);
              
//----------FUNCTION IMPLEMENTATIONS----------//
int main(int argc, char **argv)
{
  //Create a node called "robobus"
  ros::init(argc, argv, "robobus");

  //Obtain the handle of the node
  ros::NodeHandle n;

  //Create a topic to publish serial data called "serial_read" attached to the node
  ros::Publisher readTopic = n.advertise<robobus::SerialRead>("serial_read", 1000);
  
  //Create a service to obtain serial commands called "serial_write" attached to the node
  ros::ServiceServer writeService = n.advertiseService("serial_write", serialWrite);
  
  //Set the update rate of the node to be 10Hz
  ros::Rate loop_rate(500);
  
  //Initialize the serial port
  int serialPort = initSerialPort();

  //Loop while ROS system is active
  while (ros::ok())
  {
    //Publish received serial data
    updateReadOperations(serialPort, readTopic);
    
    //Transmit all data that is inside the transmit buffer along with the heartbeat signal
    updateWriteOperations(serialPort);
    
    //Sleep until next iteration
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  //Close serial port
  close(serialPort);
  
  return 0;
}


////////////////////////////////////////////////////////////////////////
// NAME: updateWriteOperations                                        //
// PURPOSE: Transmits all of the data stored in the transmit buffer.  //
//          This function should be called on every node update.      //
// PARAMETER: serialPort - The file descriptor of the serial port     //
//                         used to transmit data                      //
// RETURNS: None                                                      //
////////////////////////////////////////////////////////////////////////
void updateWriteOperations(int serialPort)
{
  char transmitArray[COMMAND_TRANSMIT_BUFFER_SIZE * 5];
  int transmitArrayHead = 0;
  
  for(int currentIndex = 0; currentIndex < currentBufferSize; currentIndex++)
  {
    transmitArray[transmitArrayHead] = requestBuffer[currentIndex].address;
    transmitArrayHead++;
    
    transmitArray[transmitArrayHead] = requestBuffer[currentIndex].command;
    transmitArrayHead++;
    
    transmitArray[transmitArrayHead] = requestBuffer[currentIndex].parameter & 0x00FF;
    transmitArrayHead++;
    
    transmitArray[transmitArrayHead] = (requestBuffer[currentIndex].parameter & 0xFF00) >> 8;
    transmitArrayHead++;
    
    transmitArray[transmitArrayHead] = transmitArray[transmitArrayHead - 1] +
                                       transmitArray[transmitArrayHead - 2] + 
                                       transmitArray[transmitArrayHead - 3] + 
                                       transmitArray[transmitArrayHead - 4];
    ROS_INFO("Transmitting bytes %i %i %i %i %i", transmitArray[transmitArrayHead],
                                                  transmitArray[transmitArrayHead - 1] ,
		                                  transmitArray[transmitArrayHead - 2] ,
					          transmitArray[transmitArrayHead - 3] ,
					          transmitArray[transmitArrayHead - 4] )
						  ;

    transmitArrayHead++;
  }

  currentBufferSize = 0;
  write(serialPort, transmitArray, transmitArrayHead);
}



////////////////////////////////////////////////////////////////////////
// NAME: updateReadOperations                                         //
// PURPOSE: Reads all data on the serial bus, parses the received     //
//          bytes, and publishes the received bytes. This function    //
//          should be called on every node update.                    //
// PARAMETER: serialPort - The file descriptor of the serial port     //
//                         used to receive data                       //
//            readTopic  - The handler for the topic that is used to  //
//                         publish the received serial data           //
// RETURNS: None                                                      //
////////////////////////////////////////////////////////////////////////
void updateReadOperations(int serialPort, ros::Publisher readTopic)
{
    static char statusBuffer[COMMAND_RECEIVE_BUFFER_SIZE * 4] = {0};
    static char statusBufferHead = 0;
    int available = 0;
    
    //Check if any new serial data has arrived
    if(ioctl(serialPort, FIONREAD, &available) < 0)
    {
      ROS_ERROR("RoboBus: Error reading serial port (buffer check failed)");
    }
    
    //If so, publish it to the SerialRead node
    if(available > 0)
    {
      //Make sure there is no buffer overflow when we read serial
      if(available > (20 - statusBufferHead)) available = 20 - statusBufferHead;
      
      //Read serial port, error out if there are issues
      if(read(serialPort, statusBuffer + statusBufferHead, available) < 0)
      {
        ROS_ERROR("RoboBus: Error reading serial port. Error %i:%s", errno, strerror(errno));
      }
     
      //Update the serial head counter
      statusBufferHead += available;
     
      //Parse received serial bytes and transmit over topic
      robobus::SerialRead msg;
      int bytesToRead = (statusBufferHead / 4) * 4;
      
      for(int index = 0; index < bytesToRead; index++)
      {
        statusBufferHead--;
      
        switch(index % 4)
        {
          //Parse address byte
          case 0:
            msg.address = statusBuffer[index];
          break; 
            
          //Parse lower byte of data
          case 1:
            msg.data = statusBuffer[index];
          break; 
           
          //Parse upper byte of data
          case 2:
            msg.data |= statusBuffer[index] << 8;
          break;
          
          //Confirm checksum. If incorrect, skip to the next byte to be processed
          case 3:
            if(statusBuffer[index] != (msg.address + (msg.data >> 8) + (msg.data & 0xFF))) return; 
            readTopic.publish(msg);
          break;
          
          default: //<--This should never execute, if so something bad is happening
          break; 
        }
      }

      //Push unparsed bytes to beginning of buffer
      for(int index = bytesToRead; index < statusBufferHead; index ++)
      {
        statusBuffer[index - bytesToRead] = statusBuffer[index];
      }
    
    }
}


////////////////////////////////////////////////////////////////////////
// NAME: initSerialPort                                               //
// PURPOSE: Obtains the handler and initalizes a serial port.         //
// PARAMETER: None                                                    //
// RETURNS: The handle for the initialized serial port                //
////////////////////////////////////////////////////////////////////////
int initSerialPort()
{
  //Open serial port
    //TODO: find how to add serial port as a command line argument
  int serialPort = open("/dev/ttyUSB0", O_RDWR);
    	
  if(serialPort < 0)
  {
    ROS_ERROR("RoboBus: Cannot open serial port. Error %i:%s", errno, strerror(errno));
    ros::shutdown();
  }
        
  //Generate and check tty attributes struct
  struct termios tty;
        
  if(tcgetattr(serialPort, &tty) != 0) {
    ROS_ERROR("RoboBus: Error getting serial port attributes");
    ros::shutdown();
  }
        
  //Set tty settings
    //No parity, will be enabled in the future
    //TODO: add parity to devices
  tty.c_cflag &= ~PARENB;
    //1 stop bit
  tty.c_cflag &= ~CSTOPB;
    //8 bits per symbol
  tty.c_cflag |= CS8;
    //disable flow control
  tty.c_cflag &= ~CRTSCTS;
    //disable carrier detect
  tty.c_cflag |= CREAD;
    //enable reading
  tty.c_cflag |= CLOCAL;
    //disable canonical mode
  tty.c_lflag &= ~(ICANON);
    //disable echo
  tty.c_lflag &= ~ECHO;
  tty.c_lflag &= ~ECHOE;
  tty.c_lflag &= ~ECHONL;
    //disable interpretation of signal charecters
  tty.c_lflag &= ~ISIG;
    //disable software flow control
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    //disable special handling of bytes
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
  tty.c_oflag &= ~OPOST;
  tty.c_oflag &= ~ONLCR;
    //disable serial timeout
  tty.c_cc[VTIME] = 1;
  tty.c_cc[VMIN] = 0;
    //set baud rate to 2400
      //TODO: find how to add baud rate as a command line argument
  cfsetispeed(&tty, B2400);
  cfsetospeed(&tty, B2400);
        
  //Set attributes to serial port
  if (tcsetattr(serialPort, TCSANOW, &tty) != 0) {
    ROS_ERROR("RoboBus: Error setting serial port attributes");
    ros::shutdown();
  }
  
  return serialPort;
}


////////////////////////////////////////////////////////////////////////
// NAME: serialWrite                                                  //
// PURPOSE: Places any serial write requests into the write buffer.   //
//          This function should be used as a callback to the serial  //
//          write service.                                            //
// PARAMETER: None                                                    //
// RETURNS: None                                                      //
////////////////////////////////////////////////////////////////////////
bool serialWrite(robobus::SerialWrite::Request &request,
                 robobus::SerialWrite::Response &response)
{
  //If buffer is not full store request in buffer and return 1, otherwise return 0.
  if(currentBufferSize < COMMAND_TRANSMIT_BUFFER_SIZE)
  {
    requestBuffer[currentBufferSize] = request;
    currentBufferSize++;
    response.success = 1;
  }
  else
  {
    response.success = 0; 
    ROS_WARN("RoboBus: Transmit buffer full, transmission request denied");
  }   
  
  return true; 
} 


