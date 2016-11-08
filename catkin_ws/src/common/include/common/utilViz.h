/**
 * utilViz.h
 * Fecha de creación: 15/03/2016, 20:06:45
 *
 * Desarrollador Reynaldo Martell Avila
 * 2016 Universidad Nacional Autónoma de México - UNAM.
 * Instituto de Investigaciones en Matemáticas Aplicadas y en Sistemas - IIMAS.
 * Laboratorio de Bio-robótica.
 * Señales Imagenes y Ambientes Virtuales.
 */
#ifndef INCLUDE_SIMULATOR_SIMULATION_UTILVIZ_H_
#define INCLUDE_SIMULATOR_SIMULATION_UTILVIZ_H_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

//#include "utilMap.h"
#include "queue.h"
 #include "definition.h"

namespace biorobotics {

void sendToVizMap(Vertex2 * vertex, bool ** adyacencies, int sizeAdyacencies,
		ros::Publisher * marker_pub) {

	visualization_msgs::Marker map_marker;
	map_marker.header.frame_id = "/map";
	map_marker.header.stamp = ros::Time::now();
	map_marker.ns = "map_markerz";
	map_marker.action = visualization_msgs::Marker::ADD;
	map_marker.pose.orientation.w = 1.0;
	map_marker.id = 2;
	map_marker.type = visualization_msgs::Marker::LINE_LIST;
	map_marker.scale.x = 0.02;
	map_marker.color.r = 0.0;
	map_marker.color.g = 0.0;
	map_marker.color.b = 1.0;
	map_marker.color.a = 1.0;
	for (int i = 0; i < sizeAdyacencies; i++) {
		for (int j = 0; j < sizeAdyacencies; j++) {
			if (adyacencies[i][j] && adyacencies[j][i]) {
				geometry_msgs::Point p;
				p.x = vertex[i].x;
				p.y = vertex[i].y;
				p.z = 0.0;
				map_marker.points.push_back(p);
				p.x = vertex[j].x;
				p.y = vertex[j].y;
				p.z = 0.0;
				map_marker.points.push_back(p);
			}
		}
	}

	marker_pub->publish(map_marker);
}

void sendToVizFloorEnvironment(
		std::vector<biorobotics::Triangle> trianglesFloor,
		ros::Publisher * marker_pub) {
	visualization_msgs::Marker floor_marker;
	floor_marker.header.frame_id = "/map";
	floor_marker.header.stamp = ros::Time::now();
	floor_marker.ns = "floor_marker";
	floor_marker.action = visualization_msgs::Marker::ADD;
	floor_marker.pose.orientation.w = 1.0;
	floor_marker.id = 3;
	floor_marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
	floor_marker.scale.x = 1.0;
	floor_marker.scale.y = 1.0;
	floor_marker.scale.z = 1.0;
	floor_marker.color.r = 0.478f;
	floor_marker.color.g = 0.478f;
	floor_marker.color.b = 0.478f;
	floor_marker.color.a = 1.0;

	for (int i = 0; i < trianglesFloor.size(); i++) {
		biorobotics::Triangle triangle = trianglesFloor[i];
		geometry_msgs::Point p1;
		geometry_msgs::Point p2;
		geometry_msgs::Point p3;
		p1.x = triangle.v1.x;
		p1.y = triangle.v1.y;
		p1.z = triangle.v1.z;
		p2.x = triangle.v2.x;
		p2.y = triangle.v2.y;
		p2.z = triangle.v2.z;
		p3.x = triangle.v3.x;
		p3.y = triangle.v3.y;
		p3.z = triangle.v3.z;

		floor_marker.points.push_back(p1);
		floor_marker.points.push_back(p2);
		floor_marker.points.push_back(p3);
	}

	marker_pub->publish(floor_marker);

}

void sendToVizEnvironment(biorobotics::Triangle * triangulationPtr,
		int num_triangles, ros::Publisher * marker_pub, std::string ns, float r, float g, float b) {

	/*Marker Environment*/
	visualization_msgs::Marker objects_marker;
	objects_marker.header.frame_id = "/map";
	objects_marker.header.stamp = ros::Time::now();
	objects_marker.ns = ns;
	objects_marker.action = visualization_msgs::Marker::ADD;
	objects_marker.pose.position.z = 0.0;
	objects_marker.pose.orientation.w = 1.0;
	objects_marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
	objects_marker.scale.x = 1.0;
	objects_marker.scale.y = 1.0;
	objects_marker.scale.z = 1.0;
	/*objects_marker.color.r = 0.6f;
	objects_marker.color.g = 0.4f;
	objects_marker.color.b = 0.2f;*/
	objects_marker.color.r = r;
	objects_marker.color.g = g;
	objects_marker.color.b = b;
	objects_marker.color.a = 1.0;

	/*Marker Environment*/
	visualization_msgs::Marker walls_marker;
	walls_marker.header.frame_id = "/map";
	walls_marker.header.stamp = ros::Time::now();
	walls_marker.ns = "walls_marker";
	walls_marker.action = visualization_msgs::Marker::ADD;
	walls_marker.pose.position.z = 0.0;
	walls_marker.pose.orientation.w = 1.0;
	walls_marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
	walls_marker.scale.x = 1.0;
	walls_marker.scale.y = 1.0;
	walls_marker.scale.z = 1.0;
	walls_marker.color.r = 0.478f;
	walls_marker.color.g = 0.478f;
	walls_marker.color.b = 0.478f;
	walls_marker.color.a = 1.0;

	for (int i = 0; i < num_triangles; i++) {
		biorobotics::Triangle triangle = triangulationPtr[i];

		geometry_msgs::Point p1;
		p1.x = triangle.v1.x;
		p1.y = triangle.v1.y;
		p1.z = triangle.v1.z;

		geometry_msgs::Point p2;
		p2.x = triangle.v2.x;
		p2.y = triangle.v2.y;
		p2.z = triangle.v2.z;

		geometry_msgs::Point p3;
		p3.x = triangle.v3.x;
		p3.y = triangle.v3.y;
		p3.z = triangle.v3.z;

		if (triangle.objectType == OBSTACLE) {
			objects_marker.points.push_back(p1);
			objects_marker.points.push_back(p2);
			objects_marker.points.push_back(p3);
		} else {
			walls_marker.points.push_back(p1);
			walls_marker.points.push_back(p2);
			walls_marker.points.push_back(p3);
		}
	}
	if (objects_marker.points.size() > 0)
		marker_pub->publish(objects_marker);
	if (walls_marker.points.size() > 0)
		marker_pub->publish(walls_marker);
}

void sendToVizSegments(std::vector<biorobotics::Segment> segments, std::string ns, ros::Publisher * marker_pub){
	visualization_msgs::Marker map_marker;
	map_marker.header.frame_id = "/map";
	map_marker.header.stamp = ros::Time::now();
	map_marker.ns = ns;
	map_marker.action = visualization_msgs::Marker::ADD;
	map_marker.pose.orientation.w = 1.0;
	map_marker.id = 2;
	map_marker.type = visualization_msgs::Marker::LINE_LIST;
	map_marker.scale.x = 0.02;
	map_marker.color.r = 0.0;
	map_marker.color.g = 0.0;
	map_marker.color.b = 1.0;
	map_marker.color.a = 1.0;

	for(int i = 0; i < segments.size(); i++){
		biorobotics::Segment segment = segments[i];
		geometry_msgs::Point p;
		p.x = segment.v1.x;
		p.y = segment.v1.y;
		p.z = 0.0;
		map_marker.points.push_back(p);
		p.x = segment.v2.x;
		p.y = segment.v2.y;
		p.z = 0.0;
		map_marker.points.push_back(p);
	}

	marker_pub->publish(map_marker);
}

void sendToVizLocationMarker(std::map<std::string, std::vector<float> > locations, 
		ros::Publisher * marker_pub){
	visualization_msgs::Marker m;
    m.ns = "locations_marker";
    m.header.frame_id = "map";
    m.type = visualization_msgs::Marker::CUBE_LIST;
    m.action = visualization_msgs::Marker::ADD;
    m.lifetime = ros::Duration();
    m.pose.orientation.x = 0;
    m.pose.orientation.y = 0;
    m.pose.orientation.z = 0;
    m.color.r = 0.0;
    m.color.g = 1.0;
    m.color.b = 1.0;
    m.color.a = 1.0;
    m.scale.x = 0.15;
    m.scale.y = 0.15;
    m.scale.z = 0.15;
    int id = 0;
    for(std::map<std::string, std::vector<float> >::iterator it=locations.begin(); 
    		it != locations.end(); it++){
        m.id = id;
        geometry_msgs::Point p;
        p.x = it->second[0];
        p.y = it->second[1];
        p.z = 0.075;
        m.points.push_back(p);
    }
    marker_pub->publish(m);
}

void sendToVizPolygonMarker(std::vector<biorobotics::Polygon> polygons, std::string ns,
		ros::Publisher * marker_pub){
	visualization_msgs::Marker map_marker;
	map_marker.header.frame_id = "/map";
	map_marker.header.stamp = ros::Time::now();
	map_marker.ns = ns;
	map_marker.action = visualization_msgs::Marker::ADD;
	map_marker.pose.orientation.w = 1.0;
	map_marker.id = 2;
	map_marker.type = visualization_msgs::Marker::LINE_LIST;
	map_marker.scale.x = 0.02;
	map_marker.color.r = 1.0;
	map_marker.color.g = 0.0;
	map_marker.color.b = 1.0;
	map_marker.color.a = 1.0;

	for (int i = 0; i < polygons.size(); i++) {
 		Polygon polygon = polygons[i];
 		for (int j = 0; j < polygon.num_vertex; j++) {
 			Vertex2 vertex1 = polygon.vertex[j];
 			Vertex2 vertex2;
	 		if (j < polygon.num_vertex - 1)
	 			vertex2 = polygon.vertex[j + 1];
	 		else
	 			vertex2 = polygon.vertex[0];
			geometry_msgs::Point p;
			p.x = vertex1.x;
			p.y = vertex1.y;
			p.z = 0.0;
			map_marker.points.push_back(p);
			p.x = vertex2.x;
			p.y = vertex2.y;
			p.z = 0.0;
			map_marker.points.push_back(p);
	 	}
	 }

	marker_pub->publish(map_marker);
}

}

#endif /* INCLUDE_SIMULATOR_SIMULATION_UTILVIZ_H_ */
