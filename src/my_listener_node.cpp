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

    linear = msg->linear.x; // m/s
    angular = msg->angular.z; // rads/s

    sc.DWrite1R(0, 88112, 1); //manual mode
    sc.DWrite1R(0, 63609, linear*1000*60); // linear.v (mm/Min)    
	sc.DWrite1R(0, 88114, 10000); // acceleration ratio in 0.01%
	sc.DWrite1R(0, 63610, angular*57.2957795);   // omega(Deg/Min)
    sc.DWrite1R(0, 88115, 10000); // acceleration ratio in 0.01%
}

//void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
//{
//  ROS_INFO("Received a /scan message with range data:");
//  float min = 1000000;
//  for (int i = 0; i < msg->ranges.size(); i++) {
//    if(msg->ranges[i] > 0 && msg->ranges[i] < min)
//        min = msg->ranges[i];
//        
//  }
//  ROS_INFO("%d", msg->ranges.size());

  //if(min>2)
    //ROS_INFO("Min distance  =: %f", min);
    // give /cmd_vel to robot
  //else
    //ROS_INFO("Min distance  =: %f < 2 !!!! cmd_vel = 0", min);
//}

int main(int argc, char** argv)
{
  if(connectRobot())
  {

      ros::init(argc, argv, "my_listener_node");

      ros::NodeHandle nh;

      // Subscribe to /cmd_vel topic
      ros::Subscriber cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, cmdVelCallback);

      // Subscribe to /scan topic
      //ros::Subscriber scan_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 10, scanCallback);

      

      // Spin the node and process incoming messages
      ros::spin();
  }
  return 0;
}
