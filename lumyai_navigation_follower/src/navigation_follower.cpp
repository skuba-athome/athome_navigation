#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_global_planner.h>
#include <nav_core/base_local_planner.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/String.h>
#include <pluginlib/class_loader.h>
// classes wich are parts of this pkg
#include <eband_local_planner/eband_local_planner.h>
#include <eband_local_planner/conversions_and_types.h>
#include <eband_local_planner/eband_visualization.h>
#include <base_local_planner/goal_functions.h>
// boost classes
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
//-----------------------
#include <lumyai_navigation_msgs/NavGoalMsg.h>

#define GOAL_RADIUS	0.5f

void pubRobotPath();
void pubTargetPath();

ros::Publisher pub_vel,pub_target_path,pub_robot_path,pub_pan_tilt,pub_global_path,pub_robot_pose,pub_is_fin;

costmap_2d::Costmap2DROS* planner_costmap_ros;
nav_core::BaseGlobalPlanner* planner;
nav_core::BaseLocalPlanner* tc;
boost::shared_ptr<eband_local_planner::EBandPlanner> eband;
boost::shared_ptr<eband_local_planner::EBandVisualization> eband_visual;

tf::TransformListener* listener;
const std::string robot_frame = "base_link";
const std::string world_frame = "odom";

static geometry_msgs::Twist::Ptr joy_cmd_vel(new geometry_msgs::Twist);

static bool pass_goalcb = false;
static bool needGbPlan = false;
static geometry_msgs::PoseStamped::Ptr goal(new geometry_msgs::PoseStamped);
static bool robot_stop = false;
static bool clear_costmap = false;

void goalCallback(const geometry_msgs::PoseStamped::Ptr& new_goal)
{
  	try{
  		listener->waitForTransform(world_frame, new_goal->header.frame_id, new_goal->header.stamp, ros::Duration(1.0));
  		listener->transformPose(world_frame,*new_goal,*goal);
		pass_goalcb = true;
  	}
  	catch(tf::TransformException& ex){
		ROS_ERROR("Received an exception trying to transform a point from %s to %s: %s", new_goal->header.frame_id.c_str(),world_frame.c_str(),ex.what());
  	}
} 

void goalrvizCallback(const geometry_msgs::PoseStamped::Ptr& new_goal)
{
	needGbPlan = true;
	pass_goalcb = true;
	goal = new_goal;
}
void cmdCallback(const lumyai_navigation_msgs::NavGoalMsg::Ptr& new_goal)
{
	if(strcmp(new_goal->text_msg.data(),"stop") == 0) 
	{
		robot_stop = true;
	}
	else robot_stop = false;
	//if(new_goal->text_msg == "clear") clear_costmap = true;
	//else clear_costmap = false;
	static geometry_msgs::PoseStamped::Ptr goal_temp(new geometry_msgs::PoseStamped);
	goal_temp->header.stamp = ros::Time::now();
	if(new_goal->ref_frame == "relative") goal_temp->header.frame_id = robot_frame;
	else if(new_goal->ref_frame == "absolute") goal_temp->header.frame_id = world_frame;
	goal_temp->pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0,new_goal->pose2d.theta);
	goal_temp->pose.position.x = new_goal->pose2d.x;
	goal_temp->pose.position.y = new_goal->pose2d.y;

  	try{
  		listener->waitForTransform(world_frame, goal_temp->header.frame_id, goal_temp->header.stamp, ros::Duration(1.0));
  		listener->transformPose(world_frame,*goal_temp,*goal);
		pass_goalcb = true;
  	}
  	catch(tf::TransformException& ex){
		ROS_ERROR("Received an exception trying to transform a point from %s to %s: %s", goal_temp->header.frame_id.c_str(),world_frame.c_str(),ex.what());
  	}
}




// set global plan to wrapper and pass it to eband
bool setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan)
{
	
	// transform global plan to the map frame we are working in
	// this also cuts the plan off (reduces it to local window)
	std::vector<geometry_msgs::PoseStamped> transformed_plan;
	std::vector<int> start_end_counts (2, (int) global_plan.size()); // counts from the end() of the plan
	if(!eband_local_planner::transformGlobalPlan(*listener, global_plan, *planner_costmap_ros, planner_costmap_ros->getGlobalFrameID(), transformed_plan, start_end_counts))
	{
		// if plan could not be tranformed abort control and local planning
		ROS_WARN("Could not transform the global plan to the frame of the controller");
		return false;
	}

    // also check if there really is a plan
	if(transformed_plan.empty())
	{
		// if global plan passed in is empty... we won't do anything
		ROS_WARN("Transformed plan is empty. Aborting local planner!");
		return false;
	}


	// set plan - as this is fresh from the global planner robot pose should be identical to start frame
	if(!eband->setPlan(transformed_plan))
	{
		ROS_ERROR("Setting plan to Elastic Band method failed!");
		return false;
	}
	ROS_INFO("Global plan set to elastic band for optimization");

	// let eband refine the plan before starting continuous operation (to smooth sampling based plans)
	eband->optimizeBand();

    return true;
}

//goal must be in world frame
void trajectory_con()
{
	//pubTargetPath();
	//pubRobotPath();
	if(!pass_goalcb) return;
	
	std::vector<geometry_msgs::Point> clear_poly;
	double x = goal->pose.position.x;
	double y = goal->pose.position.y;
	geometry_msgs::Point pt;
	
	pt.x = x - GOAL_RADIUS;
	pt.y = y - GOAL_RADIUS;
	clear_poly.push_back(pt);
	
	pt.x = x + GOAL_RADIUS;
	pt.y = y - GOAL_RADIUS;
	clear_poly.push_back(pt);
	
	pt.x = x + GOAL_RADIUS;
	pt.y = y + GOAL_RADIUS;
	clear_poly.push_back(pt);
	
	pt.x = x - GOAL_RADIUS;
	pt.y = y + GOAL_RADIUS;
	clear_poly.push_back(pt);
	
	planner_costmap_ros->setConvexPolygonCost(clear_poly, costmap_2d::FREE_SPACE);
	
	costmap_2d::Costmap2D my_costmap;
	geometry_msgs::PoseStamped newgoal;
	planner_costmap_ros->getCostmapCopy(my_costmap);
	
	newgoal = *goal;
	unsigned int px,py;
	my_costmap.worldToMap(newgoal.pose.position.x, newgoal.pose.position.y, px,py);
	if(my_costmap.getCost(px,py)>=costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
	{
		float goal_ang = atan2(goal->pose.position.y,goal->pose.position.x);
		for(float n = goal_ang; n<=goal_ang+M_PI; n+=M_PI/10)
		{
			newgoal.pose.position.x = goal->pose.position.x - GOAL_RADIUS*cos(n);
			newgoal.pose.position.y = goal->pose.position.y - GOAL_RADIUS*sin(n);
			my_costmap.worldToMap(newgoal.pose.position.x, newgoal.pose.position.y, px,py);
			if(my_costmap.getCost(px,py)<costmap_2d::INSCRIBED_INFLATED_OBSTACLE) break;
		}
	}
	
	tf::Stamped<tf::Pose> robot_pose_tf;
	geometry_msgs::PoseStamped robot_pose;
	
	planner_costmap_ros->getRobotPose(robot_pose_tf);
	tf::poseStampedTFToMsg(robot_pose_tf, robot_pose);


	
	
	geometry_msgs::Twist cmd_vel;
	cmd_vel.linear.x = 0.0f;
	cmd_vel.linear.y = 0.0f;
	cmd_vel.angular.z = 0.0f;
	
	bool plan_status = true;
	static std::vector<geometry_msgs::PoseStamped> path;
	static std::vector<eband_local_planner::Bubble> current_band;
	static std::vector<geometry_msgs::PoseStamped> refined_plan;
	
	float ang_check = tf::getYaw(robot_pose.pose.orientation)-tf::getYaw(newgoal.pose.orientation);
	if(ang_check > M_PI) ang_check-=2*M_PI;
	else if (ang_check < -M_PI) ang_check+=2*M_PI;
	std_msgs::String is_fin_temp;
	if(fabs(robot_pose.pose.position.x-newgoal.pose.position.x) < 0.4f && fabs(robot_pose.pose.position.y-newgoal.pose.position.y) < 0.4f && fabs(ang_check) < 0.5f)
	{	
		is_fin_temp.data = "SUCCEEDED";
	}else is_fin_temp.data = "ACTIVE";
	pub_is_fin.publish(is_fin_temp);

	geometry_msgs::Pose2D robot_pose2d;
	robot_pose2d.x = robot_pose.pose.position.x;
	robot_pose2d.y = robot_pose.pose.position.y;
	robot_pose2d.theta = tf::getYaw(robot_pose.pose.orientation);
	pub_robot_pose.publish(robot_pose2d);
	

	/*if(needGbPlan)
	{
		if(planner->makePlan(robot_pose,newgoal,path))
		{
			if(setPlan(path) && eband->getBand(current_band) && eband->getPlan(refined_plan))
			{	
				needGbPlan = false;	
			}
			else
			{
				ROS_WARN("Could not set a plan.........");
				plan_status = false;	
			}	
		}
		else
		{
			ROS_WARN("Could not make a plan.........");
			plan_status = false;
		}	
	}*/
	if(1)//needGbPlan)
	{
		if(planner->makePlan(robot_pose,newgoal,path))
		{
			plan_status = true;
			needGbPlan = false;
		}
		else
		{
			plan_status = false;
			ROS_WARN("Cannot find any path.........");
		}
		
		/*if(plan_status && setPlan(path) && eband->getBand(current_band) && eband->getPlan(refined_plan))
		{
			plan_status = true;
			needGbPlan = false;
		}
		else
		{
			plan_status = false;
			ROS_WARN("Cannot correct given path by eband.........");
			refined_plan = path;
		}*/
		refined_plan = path;
	}	

	// display result
	//eband_visual->publishBand("bubbles", current_band);
	//base_local_planner::publishPlan(refined_plan, pub_global_path, 0.0, 1.0, 0.0, 0.0);

	if(plan_status && tc->setPlan(refined_plan))
	{
		plan_status = true;
	}
	else
	{
		plan_status = false;
		needGbPlan = true;
		ROS_WARN("Cannot set given path to local planner.........");
	}
	
	if(plan_status && tc->computeVelocityCommands(cmd_vel))
	{
		plan_status = true;
	}
	else
	{
		needGbPlan = true;
		ROS_WARN("Cannot generate velocity.........");
	}

	if(robot_stop)
	{
		cmd_vel.linear.x = 0.0f;
		cmd_vel.linear.y = 0.0f;
		cmd_vel.angular.z = 0.0f;

		planner_costmap_ros->resetMapOutsideWindow(0,0);
	} 
	
	geometry_msgs::Twist final_cmd_vel;
	
	if(1)//(int)joy_cmd_vel->angular.x == 1)
	{
		final_cmd_vel = cmd_vel;
	}
	else
	{
		final_cmd_vel = *joy_cmd_vel;
		ROS_INFO("-----------------Joy command is enable------------------");
	}
	pub_vel.publish(final_cmd_vel);
	
	ROS_INFO("Command velocity: %.2f %.2f %.2f m/s",final_cmd_vel.linear.x,final_cmd_vel.linear.y,final_cmd_vel.angular.z);
	
	//==============================================================
	/*static float tilt_ang_filter = 0.0f,pan_ang_filter = 0.0f;
	float norm_finalvel = sqrt(final_cmd_vel.linear.x*final_cmd_vel.linear.x+final_cmd_vel.linear.y*final_cmd_vel.linear.y);
	float tilt_ang = 0.6 - norm_finalvel*0.0f;
	tilt_ang_filter = tilt_ang_filter + 0.2f*(tilt_ang - tilt_ang_filter);
	if(tilt_ang_filter>M_PI/2) tilt_ang_filter = M_PI/2;
	else if(tilt_ang_filter<-M_PI/2) tilt_ang_filter =-M_PI/2;
	
	float pan_ang = 1.0f*final_cmd_vel.angular.z;
	if(norm_finalvel>0.03f) pan_ang += atan2(final_cmd_vel.linear.y,final_cmd_vel.linear.x);
	pan_ang_filter = pan_ang_filter + 0.2f*(pan_ang - pan_ang_filter);
	if(pan_ang_filter>M_PI/2) pan_ang_filter = M_PI/2;
	else if(pan_ang_filter<-M_PI/2) pan_ang_filter =-M_PI/2;
	//===============================================================
	
	//pub_pan_tilt.publish(tf::createQuaternionMsgFromRollPitchYaw(0.0, tilt_ang_filter, pan_ang_filter));
	ROS_INFO("Command velocity: %.2f %.2f %.2f m/s, Pan: %.2f deg/s",final_cmd_vel.linear.x,final_cmd_vel.linear.y,final_cmd_vel.angular.z,pan_ang_filter*180.0/M_PI);*/
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "base_planner");
	ros::NodeHandle n;

	listener = new tf::TransformListener();
	
    planner_costmap_ros = new costmap_2d::Costmap2DROS("costmap", *listener);

	pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader("nav_core", "nav_core::BaseGlobalPlanner");
	
	planner = bgp_loader.createClassInstance("navfn/NavfnROS");//carrot_planner/CarrotPlanner");//"navfn/NavfnROS"
	planner->initialize(bgp_loader.getName("navfn/NavfnROS"), planner_costmap_ros);

	pluginlib::ClassLoader<nav_core::BaseLocalPlanner> blp_loader("nav_core", "nav_core::BaseLocalPlanner");
	tc = blp_loader.createClassInstance("base_local_planner/TrajectoryPlannerROS");//"eband_local_planner/EBandPlannerROS"//base_local_planner/TrajectoryPlannerROS
	tc->initialize(blp_loader.getName("base_local_planner/TrajectoryPlannerROS"), listener, planner_costmap_ros);//dwa_local_planner/DWAPlannerROS
	
	eband = boost::shared_ptr<eband_local_planner::EBandPlanner>(new eband_local_planner::EBandPlanner("eband_planner", planner_costmap_ros));
	eband_visual = boost::shared_ptr<eband_local_planner::EBandVisualization>(new eband_local_planner::EBandVisualization);
	
	eband->setVisualization(eband_visual);
	eband_visual->initialize(n, planner_costmap_ros);

	
	//ros::Timer timer = n.createTimer(ros::Duration(0.1), boost::bind(&trajectory_con, boost::ref(tf)));
	ros::Timer timer = n.createTimer(ros::Duration(0.1), boost::bind(&trajectory_con));
	ros::Subscriber goal_rviz = n.subscribe("move_base_simple/goal", 1, goalrvizCallback);
	ros::Subscriber goal_target = n.subscribe("target_pose", 1, goalCallback);
	ros::Subscriber sub_setpos = n.subscribe("base/set_pos", 1, cmdCallback);	

	pub_vel = n.advertise<geometry_msgs::Twist>("cmd_vel",1);
	pub_target_path = n.advertise<nav_msgs::Path>("target_path", 1);
	pub_robot_path = n.advertise<nav_msgs::Path>("robot_path", 1);
	pub_global_path = n.advertise<nav_msgs::Path>("global_path", 1);
	pub_pan_tilt = n.advertise<geometry_msgs::Quaternion>("pan_tilt_cmd", 1);
	pub_robot_pose = n.advertise<geometry_msgs::Pose2D>("base/base_pos", 1);
	pub_is_fin = n.advertise<std_msgs::String>("base/is_fin", 1);
	
	ros::spin();
}


void pubTargetPath()
{
	if(!pass_goalcb) return;
	const float log_dist_th = 0.1;
	static geometry_msgs::PoseStamped last_goal = *goal;
	static std::vector<geometry_msgs::PoseStamped> tgpath;
	float diff_x = goal->pose.position.x - last_goal.pose.position.x;
	float diff_y = goal->pose.position.y - last_goal.pose.position.y;
	if(diff_x*diff_x+diff_y*diff_y > log_dist_th*log_dist_th)
	{
		tgpath.push_back(*goal);
		last_goal = *goal;
	}
	nav_msgs::Path gui_path;
	gui_path.poses.resize(tgpath.size());
	if(!tgpath.empty())
	{
		gui_path.header.frame_id = tgpath[0].header.frame_id;
		gui_path.header.stamp = tgpath[0].header.stamp;
	}
	else return;
	for(unsigned int i=0; i < tgpath.size(); i++){
		gui_path.poses[i] = tgpath[i];
	}
	pub_target_path.publish(gui_path);
}
void pubRobotPath()
{
	tf::Stamped<tf::Pose> robot_pose_tf;
	geometry_msgs::PoseStamped robot_pose;
	
	planner_costmap_ros->getRobotPose(robot_pose_tf);
	tf::poseStampedTFToMsg(robot_pose_tf, robot_pose);
	
	const float log_dist_th = 0.1;
	static geometry_msgs::PoseStamped last_robot_pose = robot_pose;
	static std::vector<geometry_msgs::PoseStamped> rbpath;
	float diff_x = robot_pose.pose.position.x - last_robot_pose.pose.position.x;
	float diff_y = robot_pose.pose.position.y - last_robot_pose.pose.position.y;
	if(diff_x*diff_x+diff_y*diff_y > log_dist_th*log_dist_th)
	{
		rbpath.push_back(robot_pose);
		last_robot_pose = robot_pose;
	}
	nav_msgs::Path gui_path;
	gui_path.poses.resize(rbpath.size());
	if(!rbpath.empty())
	{
		gui_path.header.frame_id = rbpath[0].header.frame_id;
		gui_path.header.stamp = rbpath[0].header.stamp;
	}
	else return;
	for(unsigned int i=0; i < rbpath.size(); i++){
		gui_path.poses[i] = rbpath[i];
	}
	pub_robot_path.publish(gui_path);
}
