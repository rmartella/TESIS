/*
 * triangulateVertex.h
 *
 *  Created on: 06/09/2015
 *      Author: rey
 */

#ifndef TRIANGULATEVERTEX_H_
#define TRIANGULATEVERTEX_H_

#include "definition.h"
#include "utilSimulator.h"

namespace biorobotics {

std::vector<biorobotics::Triangle> traingulate(
		std::vector<biorobotics::Polygon> polygons) {
	std::vector<biorobotics::Triangle> out;
	float hightObs = 0.80000000;
	float hightWalls = 4.00000000;
	float hight = 0.00000000;
	int ind1;
	int ind2;
	int ind3;
	int ind4;
	int indexInit = 0;
	for (unsigned int i = 0; i < polygons.size(); i++) {
		if (polygons[i].num_vertex >= 3) {
			if (polygons[i].objectType != WALL) {
				for (unsigned int j = 0; j < polygons[i].num_vertex; j++) {
					float det = getDeterminant(polygons[i].vertex[indexInit],
							polygons[i].vertex[indexInit + 1],
							polygons[i].vertex[indexInit + 2]);
					if (det >= 0) {
						ind3 = indexInit + 1;
						ind4 = indexInit + 2;
						break;
					} else
						indexInit++;
				}
				hight = hightObs;
			} else if (polygons[i].objectType == WALL) {
				indexInit = 0;
				ind3 = indexInit + 1;
				ind4 = indexInit + 2;
				hight = hightWalls;
			}

			for (unsigned int j = 0; j < polygons[i].num_vertex; j++) {

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

				if (j + 2 < polygons[i].num_vertex) {
					if (ind3 >= polygons[i].num_vertex)
						ind3 = 0;
					if (ind4 >= polygons[i].num_vertex)
						ind4 = 0;

					vertex11.x = polygons[i].vertex[indexInit].x;
					vertex11.y = polygons[i].vertex[indexInit].y;
					vertex11.z = 0.0;
					vertex12.x = polygons[i].vertex[ind3].x;
					vertex12.y = polygons[i].vertex[ind3].y;
					vertex12.z = 0.0;
					vertex13.x = polygons[i].vertex[ind4].x;
					vertex13.y = polygons[i].vertex[ind4].y;
					vertex13.z = 0.0;

					vertex21.x = polygons[i].vertex[indexInit].x;
					vertex21.y = polygons[i].vertex[indexInit].y;
					vertex21.z = hight;
					vertex22.x = polygons[i].vertex[ind3].x;
					vertex22.y = polygons[i].vertex[ind3++].y;
					vertex22.z = hight;
					vertex23.x = polygons[i].vertex[ind4].x;
					vertex23.y = polygons[i].vertex[ind4++].y;
					vertex23.z = hight;

					out.push_back(
							biorobotics::Triangle(vertex11, vertex12, vertex13,
									polygons[i].objectType));
					out.push_back(
							biorobotics::Triangle(vertex21, vertex22, vertex23,
									polygons[i].objectType));
				}

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
		}
	}
	return out;
}

} /* namespace biorobotics */

#endif /* TRIANGULATEVERTEX_H_ */

