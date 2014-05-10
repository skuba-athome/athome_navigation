#include "ros/ros.h"
#include "std_msgs/String.h"

#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <carrot_planner/carrot_planner.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <sstream>

void startPoseCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  std::cout << msg << std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "extend_goal");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/amcl_pose", 1, startPoseCallBack);

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {

	tf::TransformListener tf(ros::Duration(10));
	costmap_2d::Costmap2DROS costmap("my_costmap", tf);

	carrot_planner::CarrotPlanner cp;
	cp.initialize("my_carrot_planner", &costmap);

    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());


    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();

    ++count;
  }


  return 0;
}
