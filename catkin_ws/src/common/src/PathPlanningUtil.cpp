#include "common/PathPlanningUtil.h"

void PathPlanningUtil::initRosConnection(ros::NodeHandle * n){
	cltPathPlanningSymbMapDjsk =  n->serviceClient<common::PathPlanning>("path_planning_symb_djsk");
	cltPathPlanningGridMap =  n->serviceClient<common::PathPlanning>("path_planning_grid");
}

nav_msgs::Path PathPlanningUtil::planPathSymbMapDjsk(float startX, float startY, float goalX, float goalY, 
		std::vector<geometry_msgs::Polygon> polygons, bool& success){
	common::PathPlanning srv;
	geometry_msgs::Pose2D start;
	geometry_msgs::Pose2D goal;

	start.x = startX;
	start.y = startY;
	goal.x = goalX;
	goal.y = goalY;

	srv.request.start = start;
	srv.request.goal = goal;
	srv.request.polygons = polygons;

	if(cltPathPlanningSymbMapDjsk.call(srv)){
		success = true;
	}
	else{
		ROS_ERROR("Failed to call service planning path");
		success = false;
	}
	return srv.response.path;
}

nav_msgs::Path PathPlanningUtil::planPathGridMap(float startX, float startY, float goalX, float goalY, bool& success){
	common::PathPlanning srv;
	geometry_msgs::Pose2D start;
	geometry_msgs::Pose2D goal;

	start.x = startX;
	start.y = startY;
	goal.x = goalX;
	goal.y = goalY;

	srv.request.start = start;
	srv.request.goal = goal;

	if(cltPathPlanningGridMap.call(srv)){
		success = true;
	}
	else{
		ROS_ERROR("Failed to call service planning path");
		success = false;
	}
	return srv.response.path;
}