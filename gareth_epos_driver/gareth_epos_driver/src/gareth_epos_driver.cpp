//============================================================================
// Name        : gareth_epos_driver.cpp
// Author      : Gareth Griffiths
// Version     :
// Copyright   : maxon motor ag 2014
// Description : A brutally butchered up version of Hello Epos in C++, Ansi-style (haha, probably not anymore).
//				 Basically - try and use the velocity position function to drive to a postion. No homing or any smart stuff has been implemented so no random magic numbers please
//				 This should be publishing the velocity, position and current too. 

// Note		   : The shitty lack of comments are thanks to maxon, not me. I've got no idea what half of this does.
//============================================================================

#include <iostream>
#include "../include/gareth_epos_driver/Definitions.h"
#include <string.h>
#include <sstream>
#include <unistd.h>
#include <getopt.h>
#include <stdlib.h>
#include <stdio.h>
#include <list>
#include <math.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/times.h>
#include <sys/time.h>

//just ros things
#include <ros/ros.h>
#include <std_msgs/Int32.h>

typedef void* HANDLE;
typedef int BOOL;

using namespace std;

/*void* g_pKeyHandle = 0;
//unsigned short g_usNodeId = 1;
string g_deviceName;
string g_protocolStackName;
string g_interfaceName;
string g_portName;
int g_baudrate = 0;
*/
const string g_programName = "HelloEposCmd";

#ifndef MMC_SUCCESS
	#define MMC_SUCCESS 0
#endif

#ifndef MMC_FAILED
	#define MMC_FAILED 1
#endif

#ifndef MMC_MAX_LOG_MSG_SIZE
	#define MMC_MAX_LOG_MSG_SIZE 512
#endif

void  LogError(string functionName, int p_lResult, unsigned int p_ulErrorCode);
void  LogInfo(string message);
void  PrintUsage();
void  PrintHeader();
void  PrintSettings();
int   OpenDevice(unsigned int* p_pErrorCode);
int   CloseDevice(unsigned int* p_pErrorCode);
void  SetDefaultParameters();
int   ParseArguments(int argc, char** argv);
//int   DemoProfilePositionMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode);
//int   Demo(unsigned int* p_pErrorCode);


//make a ROS class, basically so that we can easily pass callback data to main()
class EposVelocity
{
public:
  EposVelocity();

  ros::NodeHandle nh_;

  ros::Publisher vel_pub_; //velocity publisher
  ros::Publisher cur_pub_; //current publisher
  ros::Publisher pos_pub_; //position publisher
  ros::Subscriber desVel_sub_; //desired velocity subscriber

  void pub_vel();

private:
  void desiredVelocityCallback(const std_msgs::Int32::ConstPtr& msg);





};

EposVelocity::EposVelocity()
{


  vel_pub_ = nh_.advertise<std_msgs::Int32>("epos/velocity", 1);
  cur_pub_ = nh_.advertise<std_msgs::Int32>("epos/current", 1);
  pos_pub_ = nh_.advertise<std_msgs::Int32>("epos/position", 1);
	//VCS_GetPositionIs to get the current postion
	//VCS_GetVelocityIs to get the velocity
	//VCS_GetCurrentIs gets the current

  desVel_sub_ = nh_.subscribe<std_msgs::Int32>("epos/desiredVelocity", 10, &EposVelocity::desiredVelocityCallback, this);

}

void EposVelocity::pub_vel()
{

//	std_msgs::Int32 vel;
//	vel.data = i;
//	vel_pub_.publish(vel);

std_msgs::Int32 vel; //velocity variable
std_msgs::Int32 cur; //current variable
std_msgs::Int32 pos; //position variable

//while(ros::ok()){
  int velocity;
  int lResult = MMC_SUCCESS; 
  unsigned int p_rlErrorCode;
  if(VCS_GetVelocityIs(g_pKeyHandle, g_usNodeId, &velocity, &p_rlErrorCode) == 0)
	{
		LogError("VCS_GetVelocityIs", lResult, p_rlErrorCode);
		lResult = MMC_FAILED;
	}
	vel.data = velocity;
	// cout << "velocity - " << vel.data <<endl;
	vel_pub_.publish(vel);

	short current;
	if(VCS_GetCurrentIs(g_pKeyHandle, g_usNodeId, &current, &p_rlErrorCode) == 0)
	{
		LogError("VCS_GetCurrentIs", lResult, p_rlErrorCode);
		lResult = MMC_FAILED;
	}
	cur.data = current;
	cur_pub_.publish(cur);

	int position;
	if(VCS_GetPositionIs(g_pKeyHandle, g_usNodeId, &position, &p_rlErrorCode) == 0)
	{
		LogError("VCS_GetPositionIs", lResult, p_rlErrorCode);
		lResult = MMC_FAILED;
	}
	pos.data = position;
	pos_pub_.publish(pos);

//desvelocity was here

 // ros::spin();
//}

}

void EposVelocity::desiredVelocityCallback(const std_msgs::Int32::ConstPtr& msg)
{

	//cout << "desiredVelocityCallback" << endl;
  //geometry_msgs::Twist twist;
  //twist.angular.z = a_scale_*joy->axes[angular_];
  //twist.linear.x = l_scale_*joy->axes[linear_];
  //vel_pub_.publish(twist);
	int lResult = MMC_SUCCESS;
	unsigned int p_rlErrorCode;
	long val;
	val = (long) msg->data;
	//cout << val << endl;
	if(VCS_MoveWithVelocity(g_pKeyHandle, g_usNodeId, val, &p_rlErrorCode) == 0)
				{
					lResult = MMC_FAILED;
					LogError("VCS_MoveWithVelocity", lResult, p_rlErrorCode);
				}

}


void PrintUsage()
{
	cout << "Usage: HelloEposCmd -h -n 1 -d deviceName -s protocolStackName -i interfaceName -p portName -b baudrate" << endl;
}

void LogError(string functionName, int p_lResult, unsigned int p_ulErrorCode)
{
	cerr << g_programName << ": " << functionName << " failed (result=" << p_lResult << ", errorCode=0x" << std::hex << p_ulErrorCode << ")"<< endl;
	//stringsteam errors;
	//errors << g_programName << ": " << functionName << " failed (result=" << p_lResult << ", errorCode=0x" << std::hex << p_ulErrorCode << ")"<< endl;
	//string s = errors.str()
	//ROS_ERROR(s);
}

void LogInfo(string message)
{
	cout << message << endl;
}

void SeparatorLine()
{
	const int lineLength = 60;
	for(int i=0; i<lineLength; i++)
	{
		cout << "-";
	}
	cout << endl;
}

void PrintSettings()
{
	stringstream msg;

	msg << "default settings:" << endl;
	msg << "node id             = " << g_usNodeId << endl;
	msg << "device name         = '" << g_deviceName << "'" << endl;
	msg << "protocal stack name = '" << g_protocolStackName << "'" << endl;
	msg << "interface name      = '" << g_interfaceName << "'" << endl;
	msg << "port name           = '" << g_portName << "'"<< endl;
	msg << "baudrate            = " << g_baudrate;

	LogInfo(msg.str());

	SeparatorLine();
}

void SetDefaultParameters()
{
	//USB
	g_usNodeId = 1;
	g_deviceName = "EPOS2"; //EPOS version
	g_protocolStackName = "MAXON SERIAL V2"; //MAXON_RS232
	g_interfaceName = "USB"; //RS232
	g_portName = "USB0"; // /dev/ttyS1
	g_baudrate = 1000000; //115200
}

int OpenDevice(unsigned int* p_pErrorCode)
{
	int lResult = MMC_FAILED;

	char* pDeviceName = new char[255];
	char* pProtocolStackName = new char[255];
	char* pInterfaceName = new char[255];
	char* pPortName = new char[255];

	strcpy(pDeviceName, g_deviceName.c_str());
	strcpy(pProtocolStackName, g_protocolStackName.c_str());
	strcpy(pInterfaceName, g_interfaceName.c_str());
	strcpy(pPortName, g_portName.c_str());

	LogInfo("Open device...");

	g_pKeyHandle = VCS_OpenDevice(pDeviceName, pProtocolStackName, pInterfaceName, pPortName, p_pErrorCode);

	if(g_pKeyHandle!=0 && *p_pErrorCode == 0)
	{
		unsigned int lBaudrate = 0;
		unsigned int lTimeout = 0;

		if(VCS_GetProtocolStackSettings(g_pKeyHandle, &lBaudrate, &lTimeout, p_pErrorCode)!=0)
		{
			if(VCS_SetProtocolStackSettings(g_pKeyHandle, g_baudrate, lTimeout, p_pErrorCode)!=0)
			{
				if(VCS_GetProtocolStackSettings(g_pKeyHandle, &lBaudrate, &lTimeout, p_pErrorCode)!=0)
				{
					if(g_baudrate==(int)lBaudrate)
					{
						lResult = MMC_SUCCESS;
					}
				}
			}
		}
	}
	else
	{
		g_pKeyHandle = 0;
	}

	delete []pDeviceName;
	delete []pProtocolStackName;
	delete []pInterfaceName;
	delete []pPortName;

	return lResult;
}

int CloseDevice(unsigned int* p_pErrorCode)
{
	int lResult = MMC_FAILED;

	*p_pErrorCode = 0;

	LogInfo("Close device");

	if(VCS_CloseDevice(g_pKeyHandle, p_pErrorCode)!=0 && *p_pErrorCode == 0)
	{
		lResult = MMC_SUCCESS;
	}

	return lResult;
}

int ParseArguments(int argc, char** argv)
{
	int lOption;
	int lResult = MMC_SUCCESS;

	// Shut GetOpt error messages down (return '?'):
	opterr = 0;
	// Retrieve the options:
	while ( (lOption = getopt(argc, argv, ":hd:s:i:p:b:n:")) != -1 )
	{
		switch ( lOption ) {
			case 'h':
				PrintUsage();
				lResult = 1;
				break;
			case 'd':
				g_deviceName = optarg;
				break;
			case 's':
				g_protocolStackName = optarg;
				break;
			case 'i':
				g_interfaceName = optarg;
				break;
			case 'p':
				g_portName = optarg;
				break;
			case 'b':
				g_baudrate = atoi(optarg);
				break;
			case 'n':
				g_usNodeId = (unsigned short)atoi(optarg);
				break;
			case '?':  // unknown option...
				stringstream msg;
				msg << "Unknown option: '" << char(optopt) << "'!";
				LogInfo(msg.str());
				PrintUsage();
				lResult = MMC_FAILED;
				break;
		}
	}

	return lResult;
}

int PrepareDemo(unsigned int* p_pErrorCode)
{
	int lResult = MMC_SUCCESS;
	BOOL oIsFault = 0;

	if(VCS_GetFaultState(g_pKeyHandle, g_usNodeId, &oIsFault, p_pErrorCode ) == 0)
	{
		LogError("VCS_GetFaultState", lResult, *p_pErrorCode);
		lResult = MMC_FAILED;
	}

	if(lResult==0)
	{
		if(oIsFault)
		{
			stringstream msg;
			msg << "clear fault, node = '" << g_usNodeId << "'";
			LogInfo(msg.str());

			if(VCS_ClearFault(g_pKeyHandle, g_usNodeId, p_pErrorCode) == 0)
			{
				LogError("VCS_ClearFault", lResult, *p_pErrorCode);
				lResult = MMC_FAILED;
			}
		}

		if(lResult==0)
		{
			BOOL oIsEnabled = 0;

			if(VCS_GetEnableState(g_pKeyHandle, g_usNodeId, &oIsEnabled, p_pErrorCode) == 0)
			{
				LogError("VCS_GetEnableState", lResult, *p_pErrorCode);
				lResult = MMC_FAILED;
			}

			if(lResult==0)
			{
				if(!oIsEnabled)
				{
					if(VCS_SetEnableState(g_pKeyHandle, g_usNodeId, p_pErrorCode) == 0)
					{
						LogError("VCS_SetEnableState", lResult, *p_pErrorCode);
						lResult = MMC_FAILED;
					}
				}
			}
		}
	}
	return lResult;
}



void PrintHeader()
{
	SeparatorLine();

	LogInfo("Epos Command Library Example Program, (c) maxonmotor ag 2014-2017");

	SeparatorLine();
}

int main(int argc, char** argv)
{
	int lResult = MMC_FAILED;
	unsigned int ulErrorCode = 0;

	PrintHeader();

	SetDefaultParameters();

	if((lResult = ParseArguments(argc, argv))!=MMC_SUCCESS)
	{
		return lResult;
	}

	PrintSettings();

	if((lResult = OpenDevice(&ulErrorCode))!=MMC_SUCCESS)
	{
		LogError("OpenDevice", lResult, ulErrorCode);
		return lResult;
	}

	if((lResult = PrepareDemo(&ulErrorCode))!=MMC_SUCCESS)
	{
		LogError("PrepareDemo", lResult, ulErrorCode);
		return lResult;
	}
	lResult = MMC_SUCCESS;
	unsigned int p_rlErrorCode;
	if(VCS_ActivateProfileVelocityMode(g_pKeyHandle, g_usNodeId, &p_rlErrorCode) == 0)
	{
		LogError("VCS_ActivateProfileVelocityMode", lResult, p_rlErrorCode);
		lResult = MMC_FAILED;
	}
	ros::init(argc, argv, "epos_velocity");
	EposVelocity eposvelocity;
	while(ros::ok())
	{
 		eposvelocity.pub_vel();
	
		ros::spinOnce();
	}

	if(VCS_HaltVelocityMovement(g_pKeyHandle, g_usNodeId, &p_rlErrorCode) == 0)
			{
				lResult = MMC_FAILED;
				LogError("VCS_HaltVelocityMovement", lResult, p_rlErrorCode);
			}
	/*if((lResult = Demo(&ulErrorCode))!=MMC_SUCCESS)
	{
		LogError("Demo", lResult, ulErrorCode);
		return lResult;
	}
*/
	if((lResult = CloseDevice(&ulErrorCode))!=MMC_SUCCESS)
	{
		LogError("CloseDevice Failed", lResult, ulErrorCode);
		return lResult;
	}

	return lResult;
}
