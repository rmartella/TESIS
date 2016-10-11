#include "ros/ros.h"

#include "common/EnvironmentUtil.h"

#include "common/PathPlanning.h"
#include "common/utilMap.h"
#include "common/dijkstra.h"
#include "common/first_depth.h"
#include "common/utilViz.h"
#include "path_planning/VoronoiPlanner.h"

#include <iostream>

using namespace biorobotics;

EnvironmentUtil envu;
biorobotics::Polygon * polygons_ptr = nullptr;
int num_polygons = 0;

std::vector<biorobotics::Polygon> grownPol;

Vertex2 * vertexMap;
int sizeVertexMap = 0;
bool ** adyacencies;
int sizeAdyacencies = 0;

nav_msgs::Path lastPath;

VoronoiPlanner vp;
ros::Publisher voronoi_grid_pub_;
costmap_2d::Costmap2DROS * costmap_ros;

bool pathPlanningWithSymbolicMapDjskCallback(common::PathPlanning::Request& req,
		common::PathPlanning::Response& resp) {

	Node pathDijk;

	Vertex2 init(req.start.x, req.start.y);
	Vertex2 end(req.goal.x, req.goal.y);

	polygons_ptr = envu.convertGeometryMsgToPolygons(envu.call(), polygons_ptr,
			&num_polygons);

	std::vector<biorobotics::Polygon> polygons = std::vector<Polygon>(
			polygons_ptr, polygons_ptr + num_polygons);

	grownPol = grownPolygons(polygons, 0.34);
	adyacencies = computeMapTopologic(polygons, grownPol, 0.24,
			&sizeAdyacencies);
	vertexMap = createVertexPointerFromPolygons(&grownPol[0], num_polygons,
			&sizeVertexMap);

	/*VD * vd = computeVD(polygons);
	 vertexMap = createVertexPointerFromVD(vd, &sizeVertexMap);
	 adyacencies = computeMapTopologicVD(vd, polygons, 0.3, &sizeAdyacencies,
	 vertexMap, sizeVertexMap);*/

	adyacencies = addNodesInitEndToMap(init, end, polygons_ptr,
			polygons.size(), vertexMap, adyacencies, &sizeAdyacencies, 0.24);
	vertexMap = addVertexInitEndToVertexMap(init, end, vertexMap,
			&sizeVertexMap);
	pathDijk = dijsktra(adyacencies, sizeAdyacencies, vertexMap);

	nav_msgs::Path path;
	path.header.frame_id = "map";
	for (int i = 0; i < pathDijk.size; i++) {
		geometry_msgs::PoseStamped poseNode;
		poseNode.pose.position.x = vertexMap[pathDijk.indexPrevious[i]].x;
		poseNode.pose.position.y = vertexMap[pathDijk.indexPrevious[i]].y;
		poseNode.pose.position.z = 0.0;
		poseNode.pose.orientation.w = 1.0;
		poseNode.header.frame_id = "map";
		path.poses.push_back(poseNode);
	}

	geometry_msgs::PoseStamped poseNode;
	poseNode.pose.position.x = vertexMap[pathDijk.index].x;
	poseNode.pose.position.y = vertexMap[pathDijk.index].y;
	poseNode.pose.orientation.w = 1.0;
	poseNode.header.frame_id = "map";
	path.poses.push_back(poseNode);
	lastPath = path;
	resp.path = path;

	return true;
}

bool pathPlanningWithGridMapCallback(common::PathPlanning::Request& req,
		common::PathPlanning::Response& resp) {
	geometry_msgs::PoseStamped start;
	start.pose.position.x = req.start.x;
	start.pose.position.y = req.start.y;
	start.pose.position.z = 0;
	geometry_msgs::PoseStamped goal;
	goal.pose.position.x = req.goal.x;
	goal.pose.position.y = req.goal.y;
	goal.pose.position.z = 0;

	costmap_2d::Costmap2D * costmap_ = costmap_ros->getCostmap();
	DynamicVoronoi voronoi_;
	std::vector<geometry_msgs::PoseStamped> plan;
	vp.computePlan(costmap_, &voronoi_, start, goal, 0.01, plan);
	nav_msgs::Path path_computed = vp.makePathFromPoses(plan);

	//ROS_INFO("Size Plan %d", plan.size());

	vp.publishVoronoiGrid(&voronoi_, costmap_, costmap_ros->getGlobalFrameID(), &voronoi_grid_pub_);
	lastPath = path_computed;
	resp.path = path_computed;

	return true;
}

int main(int argc, char ** argv) {
	ros::init(argc, argv, "path_planning");
	ros::NodeHandle n;
	ros::Rate rate(30);
	ros::ServiceServer spp = n.advertiseService("path_planning_symb_djsk",
			pathPlanningWithSymbolicMapDjskCallback);
	ros::ServiceServer srvPathAStarFromAll = n.advertiseService(
			"path_planning_grid", pathPlanningWithGridMapCallback);
	ros::Publisher map_marker_pub = n.advertise<visualization_msgs::Marker>("map_markers", 10);
	ros::Publisher last_calc_path = n.advertise<nav_msgs::Path>(
			"/path_planning/last_calc_path",
			1);


	tf::TransformListener tf(ros::Duration(10));
	costmap_ros = new costmap_2d::Costmap2DROS("global_costmap", tf);
	costmap_ros->pause();
	costmap_ros->start();

	voronoi_grid_pub_ = n.advertise<nav_msgs::OccupancyGrid>("voronoi_grid", 1);

	envu.initRosConnection(&n);

	while (ros::ok()) {

		if (vertexMap != nullptr && adyacencies != nullptr)
			sendToVizMap(vertexMap, adyacencies, sizeAdyacencies,
					&map_marker_pub);
		sendToVizPolygonMarker(grownPol, "grown_polygons", &map_marker_pub);
		last_calc_path.publish(lastPath);
		rate.sleep();
		ros::spinOnce();
	}

}
