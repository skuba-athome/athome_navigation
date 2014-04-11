#include <ros/ros.h>
#include <stdio.h>
#include <string.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
unsigned int num=0;
//unsigned int count=0;
unsigned int k = 1;
bool filter_enable=false;
bool robot_move=false;
//
unsigned int check_repeat = 0;
float filter = 0.08; //0.1
unsigned int num_noise=0;
unsigned int laser_range_noise[720];
float laser_range_1[720];
float laser_range_2[720];
float laser_range_3[720];
float laser_range_4[720];
float laser_range_5[720];
unsigned int sum_laser_range_1=0;
unsigned int sum_laser_range_2=0;
unsigned int sum_laser_range_3=0;
unsigned int sum_laser_range_4=0;
unsigned int sum_laser_range_5=0;

unsigned int robot_move_frame=0;
unsigned int frame=0;

ros::Publisher pub_scan_2;
void filter_scan_enableCallback(const geometry_msgs::TwistWithCovarianceStamped filter_en)
{
	if (filter_en.twist.twist.linear.x==0&&filter_en.twist.twist.linear.y==0&&filter_en.twist.twist.angular.z==0)
	{	
		ROS_INFO("----------------ENABLE Filter--------------");
		filter_enable=true;
	}

	else
	
	{	
		ROS_INFO("----------------DISABLE Filter--------------");
		filter_enable=false;
	}
/*	if (strcmp(filter_en->data.c_str(),"Disable")==0)
	{
		filter_enable=false;
		ROS_INFO("----------------DISABLE Filter--------------");
	}
*/
}


void scanCallback(const sensor_msgs::LaserScanPtr &scan)
{	
	unsigned int ranges_size = scan->ranges.size();
//	printf("ranges_size %d",ranges_size);  Check Ranges_size
	unsigned int fin_clone = 0;
	laser_range_noise[0]=9999;



	if(k==1)
	for(unsigned int n = 0; n < ranges_size;n++)
	{
		laser_range_1[n] = scan->ranges[n];
		sum_laser_range_1=sum_laser_range_1+laser_range_1[n];
	}

	if(k==2)
	for(unsigned int n = 0; n < ranges_size;n++)
	{
		laser_range_2[n] = scan->ranges[n];
		sum_laser_range_2=sum_laser_range_2+laser_range_2[n];
	}

	if(k==3)
	for(unsigned int n = 0; n < ranges_size;n++)
	{
		laser_range_3[n] = scan->ranges[n];
		sum_laser_range_3=sum_laser_range_3+laser_range_3[n];
	}
	if(k==4)
	for(unsigned int n = 0; n < ranges_size;n++)
	{
		laser_range_4[n] = scan->ranges[n];
		sum_laser_range_4=sum_laser_range_4+laser_range_4[n];
	}
	if(k==5)
	for(unsigned int n = 0; n < ranges_size;n++)
	{
		laser_range_5[n] = scan->ranges[n];
		sum_laser_range_5=sum_laser_range_5+laser_range_5[n];
		k=0;
		fin_clone=1;


	}
	k++;

	
	/*
	if(sum_laser_range_5-sum_laser_range_1<=15&&sum_laser_range_5>sum_laser_range_1&&fin_clone==1)
	{
		printf("_____Filter-Enable______\n");
		filter_enable=true;
	}*/
	

	if(!filter_enable)
	num_noise=0;

	if(filter_enable)
	{

		if(fin_clone==1)
		{
		fin_clone=0;
		//printf("Clone complete  ************************************************* \n");

			//------------------------------- Filter -> Noise -----------------------------------------------

			for(num = 0; num < ranges_size;num++)
			{
				if(laser_range_1[num]-laser_range_2[num]>filter || laser_range_2[num]-laser_range_1[num]>filter
				||laser_range_1[num]-laser_range_3[num]>filter || laser_range_3[num]-laser_range_1[num]>filter
				||laser_range_1[num]-laser_range_4[num]>filter || laser_range_4[num]-laser_range_1[num]>filter
				||laser_range_1[num]-laser_range_5[num]>filter || laser_range_5[num]-laser_range_1[num]>filter
				||laser_range_2[num]-laser_range_4[num]>filter || laser_range_4[num]-laser_range_2[num]>filter)  
				{
					//ROS_INFO("Seq Noise[] = %d \n",num);	
					//laser_range_noise[num]=num;

				//------------------- Check repeat  ---------------------------------
					for(unsigned int j=0;j<num_noise;j++)     ///Check repeat
					{	
						
						if(laser_range_noise[j]==num)    
						{
							check_repeat=1;
						}
						//printf("laser_range_noise[%d] = %d \n",j,laser_range_noise[j]);
					}

					if(check_repeat==0)
					{
						laser_range_noise[num_noise]=num;
						//printf("laser_range_noise[%d] = %d \n",num_noise,num);
						num_noise++;
						//scan->ranges[num]=0.0;
					}
				check_repeat=0;
				
				//--------------------------------------------------------------------	
				}
			}
			///-------------------------------------------------------------------------------------------------
	
		}  

			
	//	filter_enable=true;

		for(unsigned int j=1;j<num_noise;j++)     ///Check repeat	
		{		
			scan->ranges[laser_range_noise[j]]=0.0;	
	//		printf("laser_range_noise[%d] = %d \n",j,laser_range_noise[j]);
		//	printf("num_noise = %d \n",num_noise);
		}
	}

	if(sum_laser_range_5>0)
	{
	//	printf("sum_laser_range_1 = %d \n",sum_laser_range_1);
	//	printf("sum_laser_range_2 = %d \n",sum_laser_range_2);
	//	printf("sum_laser_range_3 = %d \n",sum_laser_range_3);
	//	printf("sum_laser_range_4 = %d \n",sum_laser_range_4);
	//	printf("sum_laser_range_5 = %d \n",sum_laser_range_5);
		sum_laser_range_1=0;
		sum_laser_range_2=0;
		sum_laser_range_3=0;
		sum_laser_range_4=0;
		sum_laser_range_5=0;
	}

	pub_scan_2.publish(scan);  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "filter_scan");
  ros::NodeHandle n;
  ros::Subscriber filter_scan = n.subscribe("/scan", 1, scanCallback);
  ros::Subscriber filter_scan_enable = n.subscribe("/vel_odom",5, filter_scan_enableCallback);
//  ros::Subscriber filter_scan_enable = n.subscribe("/dear",5, filter_scan_enableCallback);
  pub_scan_2 = n.advertise<sensor_msgs::LaserScan>("/scan_2", 50);
  ros::spin();
  return 0;
}
