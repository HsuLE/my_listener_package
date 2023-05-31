#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "scif2_if.h"

SC2 sc;

//Needed to be declared here or it shows "error: ‘DllSetting’ was not declared in this scope"
DLL_USE_SETTING DllSetting;

float linear = 0;
float angular = 0;
float d = 0;
int oConnStatus=0;

int connectRobot()
{
    int n;
    if (LoadFunction("/home/lnc/Example_LinuxGCC/Example_LinuxGCC/libSCIF2_arm.so", &sc)!=0) //exit(0);
    //-----初始化函式庫
	//-----Initial Library
    //DLL_USE_SETTING DllSetting;

    DllSetting.SoftwareType = 1;
    DllSetting.ConnectNum = 1;
    DllSetting.MemSizeI  = 0;
    DllSetting.MemSizeO  = 4096;
    DllSetting.MemSizeC  = 0;
    DllSetting.MemSizeS  = 4096;
    DllSetting.MemSizeA  = 4096;
    DllSetting.MemSizeR  = 110000;  //為節省內存，只宣告110000個 R值的鏡射記憶體空間
    DllSetting.MemSizeF  = 0;

    int rt = sc.LibraryInitial(&DllSetting, 706995, "C4B6F4BBD567825AE16BB5E10F44B1426EDEF7F4B13D9FC2");
	printf("LibraryInitial rt=%d\n", rt);                
	if (rt!=100)
	{
		printf("Initial Library Failed! rt=%d\n", rt);
		return 0;
	}

	//----設定連線控制器
	//----Set the controller IP 
	int ok;  
	ok = sc.ConnectLocalIP(0, "192.168.2.113");
	if(ok != 1)
	{
		printf("Connection Setting Failed!\n");
		return 0;
	}
    printf("%d",ok);


	//-----設定要持續讀取的資料
	sc.LClearQueue(0);
	//-----Set data for continuous reading
    sc.LReadBegin(0);    //Set Auto Combination package Start
    sc.LReadNS(0, 3000, 4);     //alarm and warnning
    sc.LReadNR(0, 3000, 360);   //user define data
    sc.LReadEnd(0);   //Set Auto Combination package finish

    return 1;
   
}

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    ROS_INFO("Received a /cmd_vel message with linear velocity %f and angular velocity %f", linear, angular);

    
    sc.MainProcess();
		
	//-------Reading internal status of Library
	int ProcCounter = sc.GetLibraryMsg(SCIF_PROC_COUNTER);             //library counter: add 1 every second --> to make sure library running
	int LoopCounter = sc.GetConnectionMsg(0, SCIF_LOOP_COUNT);              //polling cycle time --> to know how fast the speed is
	int LoopNum = sc.GetConnectionMsg(0, SCIF_LOOP_QUEUE_PKG_COUNT);        //polling package count --> to make sure setting is correct
	int DirectNum = sc.GetConnectionMsg(0, SCIF_DIRECT_QUEUE_PKG_COUNT);    //direct package count not executed.

	//------Read connect status, "3" means connection OK
	int ConnStatus = sc.GetConnectionMsg(0, SCIF_CONNECT_STATE);
	if (ConnStatus!=oConnStatus)
	{
		ROS_INFO("Connection Status Change to %d!\n", ConnStatus);
		oConnStatus=ConnStatus;
	}
    
    linear = msg->linear.x; // m/s
    angular = msg->angular.z; // rads/s

    sc.DWrite1R(0, 88112, 1); //manual mode
    sc.DWrite1R(0, 63613, 6000); // linear.v (mm/Min)    
	sc.DWrite1R(0, 88113, linear*10000); // acceleration ratio in 0.01%
	sc.DWrite1R(0, 63615, 600);   // omega(Deg/Min)
    sc.DWrite1R(0, 88115, angular*5.72957795*10000); // acceleration ratio in 0.01%
}

int main(int argc, char** argv)
{
  if(connectRobot())
  {

      ros::init(argc, argv, "my_listener_node");

      ros::NodeHandle nh;

      // Subscribe to /cmd_vel topic
      ros::Subscriber cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, cmdVelCallback);
      
      // Spin the node and process incoming messages
      ros::spin();
  }
  return 0;
}
