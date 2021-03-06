#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Polygon.h>

#include "common/PathPlanning.h"

class PathPlanningUtil{
public:
	void initRosConnection(ros::NodeHandle * n);
	nav_msgs::Path planPathSymbMapDjsk(float startX, float startY, float goalX, float goalY, 
			std::vector<geometry_msgs::Polygon> polygons, bool& success);
	nav_msgs::Path planPathGridMap(float startX, float startY, float goalX, float goalY, bool& success);
protected:
	ros::ServiceClient cltPathPlanningSymbMapDjsk;
	ros::ServiceClient cltPathPlanningGridMap;
};