/**
 * environment.cpp
 * Fecha de creación: 22/04/2016, 14:24:22
 *
 * Desarrollador Reynaldo Martell Avila
 * 2016 Universidad Nacional Autónoma de México - UNAM.
 * Instituto de Investigaciones en Matemáticas Aplicadas y en Sistemas - IIMAS.
 * Laboratorio de Bio-robótica.
 * Señales Imagenes y Ambientes Virtuales.
 */

#include <ros/ros.h>

#include <visualization_msgs/Marker.h>

#include <geometry_msgs/Polygon.h>

#include <common/Environment.h>

#include <common/parserFileWRL.h>
#include <common/triangulateVertex.h>
#include <common/utilViz.h>

biorobotics::Polygon * known_polygons_ptr;
biorobotics::Polygon * unknown_polygons_ptr;
biorobotics::Polygon * all_polygons_ptr;
int num_known_polygons;
int num_unknown_polygons;
int num_all_polygons;

bool allEnvironmentServiceCallback(common::Environment::Request &req,
		common::Environment::Response &resp) {
	for (int i = 0; i < num_all_polygons; i++) {
		biorobotics::Polygon polygon = all_polygons_ptr[i];
		geometry_msgs::Polygon polygonEnv;
		for (int j = 0; j < polygon.num_vertex; j++) {
			biorobotics::Vertex2 vertex = polygon.vertex[j];
			geometry_msgs::Point32 point;
			point.x = vertex.x;
			point.y = vertex.y;
			point.z = 0.0;
			polygonEnv.points.push_back(point);
		}
		resp.polygons.push_back(polygonEnv);
	}
	return true;
}

bool knownEnvironmentServiceCallback(common::Environment::Request &req,
		common::Environment::Response &resp) {
	for (int i = 0; i < num_known_polygons; i++) {
		biorobotics::Polygon polygon = known_polygons_ptr[i];
		geometry_msgs::Polygon polygonEnv;
		for (int j = 0; j < polygon.num_vertex; j++) {
			biorobotics::Vertex2 vertex = polygon.vertex[j];
			geometry_msgs::Point32 point;
			point.x = vertex.x;
			point.y = vertex.y;
			point.z = 0.0;
			polygonEnv.points.push_back(point);
		}
		resp.polygons.push_back(polygonEnv);
	}
	return true;
}

std::vector<biorobotics::Polygon> translatePolygonsToOrigin(
		std::vector<biorobotics::Polygon> polygons,
		biorobotics::Vertex2 origin) {
	std::vector<biorobotics::Polygon> polygons_translate;
	for (int i = 0; i < polygons.size(); i++) {
		biorobotics::Polygon polygon = polygons[i];
		biorobotics::Polygon polygon_translate;
		polygon_translate.num_vertex = polygon.num_vertex;
		polygon_translate.objectType = polygon.objectType;
		polygon_translate.vertex = new biorobotics::Vertex2[polygon.num_vertex];
		for (int j = 0; j < polygon.num_vertex; j++) {
			polygon_translate.vertex[j].x = polygon.vertex[j].x - origin.x;
			polygon_translate.vertex[j].y = polygon.vertex[j].y - origin.y;
		}
		polygons_translate.push_back(polygon_translate);
	}
	return polygons_translate;
}

int main(int argc, char ** argv) {

	ros::init(argc, argv, "environment_node");
	ros::NodeHandle n;

	std::string file_env, file_env_unknown;
	n.getParam("file_env", file_env);
	n.getParam("file_env_unknown", file_env_unknown);

	std::cout << "load file of world" << file_env << std::endl;
	std::cout << "load file of world unknown" << file_env_unknown << std::endl;
	std::cout << "argc:" << argc << std::endl;
	float ox = 0.0f, oy = 0.0f;
	if(argc == 3){
		ox = atof(argv[1]);
		oy = atof(argv[2]);
	}

	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>(
			"environment_markers", 10);
	ros::ServiceServer all_environment_service = n.advertiseService(
			"all_environment_service", allEnvironmentServiceCallback);
	ros::ServiceServer known_environment_service = n.advertiseService(
			"known_environment_service", knownEnvironmentServiceCallback);

	std::vector<biorobotics::Polygon> polygons_known = biorobotics::parserFile(file_env);
	std::vector<biorobotics::Polygon> polygons_unknown = biorobotics::parserFile(file_env_unknown);
	std::vector<biorobotics::Polygon> polygons_all = polygons_known;
	polygons_all.insert(polygons_all.end(), polygons_unknown.begin(), polygons_unknown.end());

	polygons_known = translatePolygonsToOrigin(polygons_known,
			biorobotics::Vertex2(ox, oy));
	polygons_unknown = translatePolygonsToOrigin(polygons_unknown,
			biorobotics::Vertex2(ox, oy));
	polygons_all = translatePolygonsToOrigin(polygons_all,
			biorobotics::Vertex2(ox, oy));

	known_polygons_ptr = polygons_known.data();
	num_known_polygons = polygons_known.size();
	unknown_polygons_ptr = polygons_unknown.data();
	num_unknown_polygons = polygons_unknown.size();
	all_polygons_ptr = polygons_all.data();
	num_all_polygons = polygons_all.size();

	std::vector<biorobotics::Triangle> known_triangulation = biorobotics::traingulate(
			polygons_known);
	biorobotics::Triangle * known_triangulationPtr = known_triangulation.data();
	int num_known_triangles = known_triangulation.size();

	std::vector<biorobotics::Triangle> unknown_triangulation = biorobotics::traingulate(
			polygons_unknown);
	biorobotics::Triangle * unknown_triangulationPtr = unknown_triangulation.data();
	int num_unknown_triangles = unknown_triangulation.size();

	std::vector<biorobotics::Triangle> trianglesFloor =
			biorobotics::makeFloorEnvironment(all_polygons_ptr, num_all_polygons);

	ROS_INFO("size: known polygons %d", num_known_polygons);
	ROS_INFO("size: known triangulation %d", num_known_triangles);
	ROS_INFO("size: known polygons %d", num_unknown_polygons);
	ROS_INFO("size: known triangulation %d", num_unknown_triangles);

	ros::Rate loop_rate(5);

	while (ros::ok()) {

		biorobotics::sendToVizEnvironment(known_triangulationPtr, num_known_triangles,
				&marker_pub, "known_objects_marker" , 1.0f, 0.5f, 0.0f);
		biorobotics::sendToVizEnvironment(unknown_triangulationPtr, num_unknown_triangles,
				&marker_pub, "unknown_objects_marker" , 1.0f, 0.0f, 0.0f);
		biorobotics::sendToVizFloorEnvironment(trianglesFloor, &marker_pub);

		loop_rate.sleep();
		ros::spinOnce();
	}
	return 1;
}
