#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <lumyai_navigation_msgs/NavGoalMsg.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/String.h>

ros::Publisher pub_scan;
geometry_msgs::Pose2D tt_point; 

void scanCallback(const sensor_msgs::LaserScanConstPtr &scan){
	unsigned int center_index = (unsigned int)((tt_point.theta - (scan->angle_min))/scan->angle_increment);
	float dist = sqrt(pow(tt_point.x,2) + pow(tt_point.y,2));
	unsigned int index_rad = atan2(0.4,dist)/scan->angle_increment;
	sensor_msgs::LaserScan scan_out = *scan;
	std::cout << center_index << ", "<< dist << ", " << index_rad << ", " << scan->ranges.size() << std::endl;
	for(unsigned int n = center_index-index_rad; n <= center_index+index_rad;n++)
	{
		scan_out.ranges[n] = 0.0f;
	}
	
	pub_scan.publish(scan_out);  
}

void targetCallback(const lumyai_navigation_msgs::NavGoalMsg::Ptr& new_goal)
{
	if(strcmp(new_goal->ref_frame.data(),"relative") != 0) return;
	
	tt_point = new_goal->pose2d;
}

int main(int argc, char **argv)
{
	tt_point.x = 1.0;
	tt_point.y = 0.0;
	tt_point.theta = 0.0;
  ros::init(argc, argv, "door_detection");

  ros::NodeHandle n;
  ros::Subscriber scan_sub = n.subscribe("/scan", 1, scanCallback);
	ros::Subscriber sub_point = n.subscribe("/follow/point", 1, targetCallback);
  pub_scan = n.advertise<sensor_msgs::LaserScan> ("/scan_remove_target", 1);
  ros::spin();
  
  return 0;
}
