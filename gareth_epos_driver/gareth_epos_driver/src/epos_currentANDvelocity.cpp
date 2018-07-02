//============================================================================
// Name        : salua_epos_driver.cpp
// Author      : Salua Hamaza
// Version     :
// Copyright   : maxon motor ag 2014
// Description : Current Control from Linux library from Maxon
//============================================================================

#include <iostream>
#include <string>
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
#include <ros/ros.h>  //ROS stuff
#include <std_msgs/Int32.h>  //ROS stuff

typedef void* HANDLE;
typedef int BOOL;

using namespace std;
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

//ROS class, so that we can easily pass callback data to main()
class EposController
{
public:
  EposController();

  ros::NodeHandle nh_;

  ros::Publisher vel_pub_;        //velocity publisher
  ros::Publisher cur_pub_;        //current publisher
  ros::Publisher pos_pub_;        //position publisher
  ros::Subscriber desCurr_sub_;   //desired current subscriber
  ros::Subscriber desVel_sub_;    //desired velocity subscriber
  ros::Subscriber eposMode_sub_;  //epos switch mode subscriber
  ros::Subscriber eposReset_sub_; //epos reset devie subscriber

  void pub_epos_states();

private:
  void desiredCurrentCallback(const std_msgs::Int32::ConstPtr& msg);
  void desiredVelocityCallback(const std_msgs::Int32::ConstPtr& msg);
  void switchModeCallback(const std_msgs::Int32::ConstPtr& msg);
  void resetDeviceCallback(const std_msgs::Int32::ConstPtr& msg);
};

EposController::EposController()
{
  vel_pub_ = nh_.advertise<std_msgs::Int32>("epos/velocity", 1);
  cur_pub_ = nh_.advertise<std_msgs::Int32>("epos/current", 1);
  pos_pub_ = nh_.advertise<std_msgs::Int32>("epos/position", 1);

  desCurr_sub_ = nh_.subscribe<std_msgs::Int32>("epos/desiredCurrent", 10, &EposController::desiredCurrentCallback, this);
  desVel_sub_ = nh_.subscribe<std_msgs::Int32>("epos/desiredVelocity", 10, &EposController::desiredVelocityCallback, this);
  eposMode_sub_ = nh_.subscribe<std_msgs::Int32>("epos/switchMode", 10, &EposController::switchModeCallback, this);
  eposReset_sub_ = nh_.subscribe<std_msgs::Int32>("epos/resetDevice", 10, &EposController::resetDeviceCallback, this);
}


void EposController::pub_epos_states()
{
	std_msgs::Int32 vel; //velocity variable
	std_msgs::Int32 cur; //current variable
	std_msgs::Int32 pos; //position variable

	int lResult = MMC_SUCCESS; 
	unsigned int p_rlErrorCode;

	int velocity;
	if(VCS_GetVelocityIs(g_pKeyHandle, g_usNodeId, &velocity, &p_rlErrorCode) == 0)
	{
		LogError("VCS_GetVelocityIs", lResult, p_rlErrorCode);
		lResult = MMC_FAILED;
	}
	vel.data = velocity;
	vel_pub_.publish(vel);

	short current;
	if(VCS_GetCurrentIsAveraged(g_pKeyHandle, g_usNodeId, &current, &p_rlErrorCode) == 0)
	{
		LogError("VCS_GetCurrentIsAveraged", lResult, p_rlErrorCode);
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
}

void EposController::desiredCurrentCallback(const std_msgs::Int32::ConstPtr& msg)
{
	int lResult = MMC_SUCCESS;
	unsigned int p_rlErrorCode;
	short val;
	val = (short) msg->data;
	if(VCS_SetCurrentMust(g_pKeyHandle, g_usNodeId, val, &p_rlErrorCode) == 0)
	{
		lResult = MMC_FAILED;
		LogError("VCS_SetCurrentMust", lResult, p_rlErrorCode);
	}
}

void EposController::desiredVelocityCallback(const std_msgs::Int32::ConstPtr& msg)
{
	int lResult = MMC_SUCCESS;
	unsigned int p_rlErrorCode;
	long val;
	val = (long) msg->data;
	if(VCS_MoveWithVelocity(g_pKeyHandle, g_usNodeId, val, &p_rlErrorCode) == 0)
	{
		lResult = MMC_FAILED;
		LogError("VCS_MoveWithVelocity", lResult, p_rlErrorCode);
	}
}


void EposController::switchModeCallback(const std_msgs::Int32::ConstPtr& msg) //when starting epos, subscribe to topic "epos/switchMode", if data == 0 activate current mode; if data ==1 activate velocity mode
{
  int lResult = MMC_SUCCESS;
  unsigned int p_rlErrorCode;
  int val;
  val = (int) msg->data;

  if(val == 0)
  {
	if(VCS_ActivateCurrentMode(g_pKeyHandle, g_usNodeId, &p_rlErrorCode) == 0)
	{
		LogError("VCS_ActivateCurrentMode", lResult, p_rlErrorCode);
		lResult = MMC_FAILED;
	}
	//LogInfo("current mode");
  }
  else if(val == 1)
  {
	if(VCS_ActivateProfileVelocityMode(g_pKeyHandle, g_usNodeId, &p_rlErrorCode) == 0)
	{
		LogError("VCS_ActivateProfileVelocityMode", lResult, p_rlErrorCode);
		lResult = MMC_FAILED;
	}
	//LogInfo("velocity mode");
  }
}


void EposController::resetDeviceCallback(const std_msgs::Int32::ConstPtr& msg)
{
  int lResult = MMC_SUCCESS;
  unsigned int p_rlErrorCode;
  int val;
  val = (int) msg->data;
  if(val == 1)
  {
	if(VCS_ResetDevice(g_pKeyHandle, g_usNodeId, &p_rlErrorCode) == 0)
	{
		LogError("VCS_ResetDevice", lResult, p_rlErrorCode);
		lResult = MMC_FAILED;
	}
	LogInfo("resetting..");
  }
}




void PrintUsage()
{
	cout << "Usage: HelloEposCmd -h -n 1 -d deviceName -s protocolStackName -i interfaceName -p portName -b baudrate" << endl;
}

void LogError(string functionName, int p_lResult, unsigned int p_ulErrorCode)
{
	cerr << g_programName << ": " << functionName << " failed (result=" << p_lResult << ", errorCode=0x" << std::hex << p_ulErrorCode << ")"<< endl;
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
		LogError("OpenDevice Failed", lResult, ulErrorCode);
		return lResult;
	}

	if((lResult = PrepareDemo(&ulErrorCode))!=MMC_SUCCESS)
	{
		LogError("PrepareDemo Failed", lResult, ulErrorCode);
		return lResult;
	}
	
	// Initialise ROS and while ros runs fine, run EposController class
	ros::init(argc, argv, "epos_activate_modes");
	EposController mycontroller;
	while(ros::ok())
	{
 		mycontroller.pub_epos_states();	
		ros::spinOnce();
	}


	if((lResult = CloseDevice(&ulErrorCode))!=MMC_SUCCESS)
	{
		LogError("CloseDevice Failed", lResult, ulErrorCode);
		return lResult;
	}

	return lResult;
}
