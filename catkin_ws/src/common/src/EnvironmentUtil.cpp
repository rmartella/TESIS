/**
 * EnvironmentClient.cpp
 * Fecha de creación: 23/04/2016, 18:10:51
 *
 * Desarrollador Reynaldo Martell Avila
 * 2016 Universidad Nacional Autónoma de México - UNAM.
 * Instituto de Investigaciones en Matemáticas Aplicadas y en Sistemas - IIMAS.
 * Laboratorio de Bio-robótica.
 * Señales Imagenes y Ambientes Virtuales.
 */
#include "common/EnvironmentUtil.h"

EnvironmentUtil::EnvironmentUtil() {
	clientAll = nullptr;
	clientKnown = nullptr;
}

EnvironmentUtil::EnvironmentUtil(ros::NodeHandle * n) {
	clientAll = new ros::ServiceClient(
			n->serviceClient<common::Environment>("/all_environment_service"));
	clientKnown = new ros::ServiceClient(
			n->serviceClient<common::Environment>("/known_environment_service"));
}

EnvironmentUtil::~EnvironmentUtil() {
	delete clientAll;
	delete clientKnown;
}

void EnvironmentUtil::initRosConnection(ros::NodeHandle * n) {
	clientAll = new ros::ServiceClient(
			n->serviceClient<common::Environment>("/all_environment_service"));
	clientKnown = new ros::ServiceClient(
			n->serviceClient<common::Environment>("/known_environment_service"));
}

std::vector<geometry_msgs::Polygon> EnvironmentUtil::getAllPolygons(){
	common::Environment srv;
	std::vector<geometry_msgs::Polygon> polygonssrv;
	if (clientAll->call(srv)) {
		polygonssrv = srv.response.polygons;
	} else {
		ROS_ERROR("Failed to call service Environment");
	}
	return polygonssrv;
}

std::vector<geometry_msgs::Polygon> EnvironmentUtil::getKnownPolygons() {
	common::Environment srv;
	std::vector<geometry_msgs::Polygon> polygonssrv;
	if (clientKnown->call(srv)) {
		polygonssrv = srv.response.polygons;
	} else {
		ROS_ERROR("Failed to call service Environment");
	}
	return polygonssrv;
}

biorobotics::Polygon * EnvironmentUtil::convertGeometryMsgToPolygons(
		std::vector<geometry_msgs::Polygon> polygonsMsg,
		biorobotics::Polygon * polygons_ptr, int * num_polygons_ptr) {
	if (polygons_ptr != nullptr) {
		for (int i = 0; i < *num_polygons_ptr; i++) {
			biorobotics::Polygon polygon = polygons_ptr[i];
			delete polygon.vertex;
		}
		delete polygons_ptr;
	}
	polygons_ptr = new biorobotics::Polygon[polygonsMsg.size()];
	*num_polygons_ptr = polygonsMsg.size();
	for (int i = 0; i < *num_polygons_ptr; i++) {
		biorobotics::Vertex2 * vertex =
				new biorobotics::Vertex2[polygonsMsg[i].points.size()];
		for (int j = 0; j < polygonsMsg[i].points.size(); j++) {
			biorobotics::Vertex2 v(polygonsMsg[i].points[j].x,
					polygonsMsg[i].points[j].y);
			vertex[j] = v;
		}
		polygons_ptr[i].num_vertex = polygonsMsg[i].points.size();
		polygons_ptr[i].vertex = vertex;
	}
	return polygons_ptr;
}

