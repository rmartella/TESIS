/**
 * VoronoiTest.h
 * Fecha de creación: 07/03/2016, 11:01:23
 *
 * Desarrollador Reynaldo Martell Avila
 * 2016 Universidad Nacional Autónoma de México - UNAM.
 * Instituto de Investigaciones en Matemáticas Aplicadas y en Sistemas - IIMAS.
 * Laboratorio de Bio-robótica.
 * Señales Imagenes y Ambientes Virtuales.
 */
#ifndef INCLUDE_SIMULATOR_SIMULATION_VORONOITEST_H_
#define INCLUDE_SIMULATOR_SIMULATION_VORONOITEST_H_

#include <visualization_msgs/Marker.h>

#include "utilSimulator.h"
#include "intersectionTest.h"

/*Include to DV*/
// includes for defining the Voronoi diagram adaptor
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Voronoi_diagram_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_traits_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_policies_2.h>
//#include <CGAL/Cartesian.h>

// typedefs for defining the adaptor
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Delaunay_triangulation_2<K> DT;
typedef CGAL::Delaunay_triangulation_adaptation_traits_2<DT> AT;
typedef CGAL::Delaunay_triangulation_caching_degeneracy_removal_policy_2<DT> AP;
typedef CGAL::Voronoi_diagram_2<DT, AT, AP> VD;

// typedef for the result type of the point location
typedef AT::Site_2 Site_2;
typedef AT::Point_2 Point_2;

typedef VD::Edge_iterator Edge_iterator;
typedef VD::Vertex_iterator Vertex_iterator;

namespace biorobotics {

visualization_msgs::Marker makeMarkerVoronoi(VD * vd) {

	visualization_msgs::Marker map_marker;
	map_marker.header.frame_id = "/map";
	map_marker.header.stamp = ros::Time::now();
	map_marker.ns = "V_map_marker";
	map_marker.action = visualization_msgs::Marker::ADD;
	map_marker.pose.orientation.w = 1.0;
	map_marker.id = 2;
	map_marker.type = visualization_msgs::Marker::LINE_LIST;
	map_marker.scale.x = 0.02;
	map_marker.color.r = 0.0;
	map_marker.color.g = 0.0;
	map_marker.color.b = 1.0;
	map_marker.color.a = 1.0;

	/*std::vector<Vertex2> centroids = computeCentroids(polygons);
	 for (int j = 0; j < centroids.size(); j++) {
	 biorobotics::Vertex2 vertex1 = centroids[j];
	 Site_2 t(vertex1.x, vertex1.y);
	 vd.insert(t);
	 }*/

	/*for (int i = 0; i < polygons.size(); i++) {
	 biorobotics::Polygon polygon = polygons[i];
	 for (int j = 0; j < polygon.num_vertex; j++) {
	 biorobotics::Vertex2 vertex1 = polygon.vertex[j];
	 Site_2 s1(vertex1.x, vertex1.y);
	 vd.insert(s1);
	 }
	 }*/

	for (Edge_iterator it = vd->edges_begin(); it != vd->edges_end(); it++) {
		if (it->is_segment()) {
			geometry_msgs::Point p;
			p.x = it->source()->point().x();
			p.y = it->source()->point().y();
			p.z = 0.0;
			map_marker.points.push_back(p);
			p.x = it->target()->point().x();
			p.y = it->target()->point().y();
			p.z = 0.0;
			map_marker.points.push_back(p);
		}
		if (it->is_ray()) {
			if (it->has_source()) {
				std::cout << "Soruce:" << it->source()->point().x() << ","
						<< it->source()->point().y() << std::endl;
			}
			if (it->ccb()->has_target()) {
				std::cout << "Target:" << it->ccb()->target()->point().x()
						<< "," << it->ccb()->target()->point().y() << std::endl;
			}
		}
	}

	return map_marker;
}

VD * computeVD(std::vector<Polygon> polygons) {
	VD * vd = new VD();
	for (int i = 0; i < polygons.size(); i++) {
		biorobotics::Polygon polygon = polygons[i];
		for (int j = 0; j < polygon.num_vertex; j++) {
			biorobotics::Vertex2 vertex1 = polygon.vertex[j];
			Site_2 s1(vertex1.x, vertex1.y);
			vd->insert(s1);
		}
	}
	std::vector<Vertex2> centroids = computeCentroids(polygons);
	for (int j = 0; j < centroids.size(); j++) {
		biorobotics::Vertex2 vertex1 = centroids[j];
		Site_2 t(vertex1.x, vertex1.y);
		//vd->insert(t);
	}
	return vd;
}

int findIndexVertexMap(Vertex2 vertex, Vertex2 * vertexMap, int sizeVertexMap) {
	int index;
	for (index = 0; index < sizeVertexMap; index++) {
		if (vertexMap[index].sub(vertex).norm() < 0.00001)
			break;
	}
	return index;
}

bool ** computeMapTopologicVD(VD * vd, std::vector<Polygon> polygons,
		float ratio, int * sizeAdyacencies, Vertex2 * vertex,
		int sizeVertexMap) {

	int totalSizeVertex = 0;
	for (Vertex_iterator it = vd->vertices_begin(); it != vd->vertices_end();
			it++) {
		totalSizeVertex++;
	}
	*sizeAdyacencies = totalSizeVertex;

	bool ** adyacencies = (bool **) malloc(totalSizeVertex * sizeof(bool *));
	for (int i = 0; i < totalSizeVertex; i++) {
		(adyacencies[i] = (bool *) malloc(totalSizeVertex * sizeof(bool)));
	}
	for (int i = 0; i < totalSizeVertex; i++) {
		for (int j = 0; j < totalSizeVertex; j++)
			adyacencies[i][j] = 0;
	}

	int indexVertex = 0;
	for (Edge_iterator it = vd->edges_begin(); it != vd->edges_end(); it++) {
		if (it->is_segment()) {
			Vertex2 v1 = Vertex2(it->source()->point().x(),
					it->source()->point().y());
			Vertex2 v2 = Vertex2(it->target()->point().x(),
					it->target()->point().y());
			Segment segment(v1, v2);
			int index1 = findIndexVertexMap(v1, vertex, sizeVertexMap);
			int index2 = findIndexVertexMap(v2, vertex, sizeVertexMap);
			/*segment.v1 = Vertex2(it->source()->point().x(),
			 it->source()->point().y());
			 segment.v2 = Vertex2(it->target()->point().x(),
			 it->target()->point().y());*/
			Segment s1;
			Segment s2;
			computeParallelLines(segment, &s1, &s2, ratio);
			bool isIntersectBool = false;
			for (int i = 0; i < polygons.size(); i++) {
				biorobotics::Polygon polygon = polygons[i];
				for (int j = 0; j < polygon.num_vertex; j++) {
					biorobotics::Vertex2 verticeTest1 = polygon.vertex[j];
					biorobotics::Vertex2 verticeTest2;
					if (j < polygon.num_vertex - 1)
						verticeTest2 = polygon.vertex[j + 1];
					else
						verticeTest2 = polygon.vertex[0];
					Segment st;
					st.v1 = verticeTest1;
					st.v2 = verticeTest2;
					if (testSegmentIntersect(segment, st)
							|| testSegmentIntersect(s1, st)
							|| testSegmentIntersect(s2, st)) {
						isIntersectBool = true;
						break;
					}
					/*if (testSegmentIntersect(segment, st)) {
					 isIntersectBool = true;
					 break;
					 }*/
				}
				if (isIntersectBool) {
					break;
				}
			}
			if (!isIntersectBool) {
				adyacencies[index1][index2] = 1;
				adyacencies[index2][index1] = 1;
			}
			indexVertex += 2;
		}
	}
	return adyacencies;
}

Vertex2 * createVertexPointerFromVD(VD * vd, int * sizeVertexMapPtr) {
	int totalSizeVertex = 0;
	for (Vertex_iterator it = vd->vertices_begin(); it != vd->vertices_end();
			it++) {
		totalSizeVertex++;
	}

	Vertex2 * vertexMap = (Vertex2 *) malloc(sizeof(Vertex2) * totalSizeVertex);
	int i = 0;
	for (Vertex_iterator it = vd->vertices_begin(); it != vd->vertices_end();
			it++) {
		vertexMap[i] = Vertex2(it->point().x(), it->point().y());
		i++;
	}
	*sizeVertexMapPtr = totalSizeVertex;
	return vertexMap;
}

}

#endif /* INCLUDE_SIMULATOR_SIMULATION_VORONOITEST_H_ */
