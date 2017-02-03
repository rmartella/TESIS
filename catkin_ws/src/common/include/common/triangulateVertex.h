/*
 * triangulateVertex.h
 *
 *  Created on: 06/09/2015
 *      Author: rey
 */

#ifndef TRIANGULATEVERTEX_H_
#define TRIANGULATEVERTEX_H_

#include <vector>
#include <list>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Regular_triangulation_euclidean_traits_2.h>
#include <CGAL/Regular_triangulation_filtered_traits_2.h>
#include <CGAL/Regular_triangulation_2.h>

#include <CGAL/Partition_traits_2.h>
#include <CGAL/partition_2.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Regular_triangulation_filtered_traits_2<K> Traits;
typedef CGAL::Regular_triangulation_2<Traits> Regular_triangulation;

typedef CGAL::Partition_traits_2<K> Par_Traits;
typedef CGAL::Is_convex_2<Traits> Is_convex_2;
typedef Par_Traits::Point_2 Point_2;
typedef Par_Traits::Polygon_2 Polygon_2;
typedef std::list<Polygon_2> Polygon_list;

typedef Polygon_2::Vertex_iterator Polygon_2_Vertex_iterator;
typedef Regular_triangulation::Finite_faces_iterator Finite_faces_iterator;

#include "definition.h"
#include "utilSimulator.h"

namespace biorobotics {

std::vector<biorobotics::Triangle> traingulate(
		std::vector<biorobotics::Polygon> polygons) {
	std::vector<biorobotics::Triangle> out;
	float hightObs = 0.80000000;
	float hightWalls = 1.750000000;
	float hight = 0.00000000;
	int ind1;
	int ind2;
	
	for (unsigned int i = 0; i < polygons.size(); i++) {
		if (polygons[i].num_vertex >= 3) {

			Polygon_2 polygon;
			Polygon_list y_monotone_partition_polys;
			Polygon_list convex_partition_polys;
			Par_Traits y_monotone_partition_traits;

			if (polygons[i].objectType != WALL) 
				hight = hightObs;
			else if (polygons[i].objectType == WALL)
				hight = hightWalls;

			for (unsigned int j = 0; j < polygons[i].num_vertex; j++) {

				polygon.push_back(
					Point_2(polygons[i].vertex[j].x, polygons[i].vertex[j].y));

				if (j + 1 < polygons[i].num_vertex) {
					ind1 = j;
					ind2 = j + 1;
				} else {
					ind1 = j;
					ind2 = 0;
				}

				Vertex3 vertex11 = Vertex3::Zero();
				Vertex3 vertex12 = Vertex3::Zero();
				Vertex3 vertex13 = Vertex3::Zero();
				Vertex3 vertex21 = Vertex3::Zero();
				Vertex3 vertex22 = Vertex3::Zero();
				Vertex3 vertex23 = Vertex3::Zero();

				vertex11.x = polygons[i].vertex[ind1].x;
				vertex11.y = polygons[i].vertex[ind1].y;
				vertex11.z = 0.0;
				vertex12.x = polygons[i].vertex[ind2].x;
				vertex12.y = polygons[i].vertex[ind2].y;
				vertex12.z = 0.0;
				vertex13.x = polygons[i].vertex[ind1].x;
				vertex13.y = polygons[i].vertex[ind1].y;
				vertex13.z = hight;

				vertex21.x = polygons[i].vertex[ind2].x;
				vertex21.y = polygons[i].vertex[ind2].y;
				vertex21.z = 0.0;
				vertex22.x = polygons[i].vertex[ind2].x;
				vertex22.y = polygons[i].vertex[ind2].y;
				vertex22.z = hight;
				vertex23.x = polygons[i].vertex[ind1].x;
				vertex23.y = polygons[i].vertex[ind1].y;
				vertex23.z = hight;

				out.push_back(
						biorobotics::Triangle(vertex11, vertex12, vertex13,
								polygons[i].objectType));
				out.push_back(
						biorobotics::Triangle(vertex21, vertex22, vertex23,
								polygons[i].objectType));
			}

			CGAL::y_monotone_partition_2(polygon.vertices_begin(),
				polygon.vertices_end(),
				std::back_inserter(y_monotone_partition_polys),
				y_monotone_partition_traits);

			CGAL::approx_convex_partition_2(polygon.vertices_begin(),
				polygon.vertices_end(),
				std::back_inserter(convex_partition_polys),
				y_monotone_partition_traits);

			std::list<Polygon_2>::const_iterator poly_it;
			for (poly_it = convex_partition_polys.begin();
				poly_it != convex_partition_polys.end(); poly_it++) {
				std::vector<Regular_triangulation::Weighted_point> wpoints;

				for (Polygon_2_Vertex_iterator ver_it = poly_it->vertices_begin();
						ver_it != poly_it->vertices_end(); ver_it++) {
					Regular_triangulation::Weighted_point wp;
					wp = Regular_triangulation::Weighted_point(ver_it->x(),
							ver_it->y());
					wpoints.push_back(wp);
				}
				Regular_triangulation rt(wpoints.begin(), wpoints.end());
				rt.is_valid();

				for (Finite_faces_iterator it = rt.finite_faces_begin();
						it != rt.finite_faces_end(); it++) {

					Vertex3 vertex11 = Vertex3::Zero();
					Vertex3 vertex12 = Vertex3::Zero();
					Vertex3 vertex13 = Vertex3::Zero();
					Vertex3 vertex21 = Vertex3::Zero();
					Vertex3 vertex22 = Vertex3::Zero();
					Vertex3 vertex23 = Vertex3::Zero();

					vertex13.x = rt.triangle(it)[0].x();
					vertex13.y = rt.triangle(it)[0].y();
					vertex13.z = 0.0;

					vertex12.x = rt.triangle(it)[1].x();
					vertex12.y = rt.triangle(it)[1].y();
					vertex12.z = 0.0;

					vertex11.x = rt.triangle(it)[2].x();
					vertex11.y = rt.triangle(it)[2].y();
					vertex11.z = 0.0;

					vertex21.x = rt.triangle(it)[0].x();
					vertex21.y = rt.triangle(it)[0].y();
					vertex21.z = hight;

					vertex22.x = rt.triangle(it)[1].x();
					vertex22.y = rt.triangle(it)[1].y();
					vertex22.z = hight;

					vertex23.x = rt.triangle(it)[2].x();
					vertex23.y = rt.triangle(it)[2].y();
					vertex23.z = hight;

					out.push_back(
							biorobotics::Triangle(vertex11, vertex12, vertex13,
									polygons[i].objectType));
					out.push_back(
							biorobotics::Triangle(vertex21, vertex22, vertex23,
									polygons[i].objectType));
				}
			}
		}
	}
	return out;
}

} /* namespace biorobotics */

#endif /* TRIANGULATEVERTEX_H_ */

