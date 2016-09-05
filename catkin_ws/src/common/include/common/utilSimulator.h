/*
 * utilSimulator.h
 *
 *  Created on: 07/09/2015
 *      Author: rey
 */

#ifndef UTILSIMULATOR_H_
#define UTILSIMULATOR_H_

#include "definition.h"

#define aisgl_min(x,y) (x<y?x:y)
#define aisgl_max(x,y) (y>x?y:x)

enum TypeMotion {
	STOP, FORWARD, BACKWARD, LEFT, RIGHT
};

namespace biorobotics {

float getDeterminant(Vertex2 vertex1, Vertex2 vertex2, Vertex2 vertex3) {
	float determinant = (vertex2.x - vertex1.x) * (vertex3.y - vertex2.y)
			- (vertex2.y - vertex1.y) * (vertex3.x - vertex2.x);
	return determinant;
}

void getConstantMovement(float xf, float tf, float * a3, float * a2) {
	*a3 = -2 * xf / pow(tf, 3);
	*a2 = 3 * xf / pow(tf, 2);
}

float getNextPosition(float a3, float a2, float t) {
	float xt = a2 * pow(t, 2) + a3 * pow(t, 3);
	return xt;
}

/*int quantizedInputs(const std::vector<simulator::LaserSensor> &sensors,
		const float angleRobot, const float rangeSensor,
		const float thresholdSensor) {
	float sd = 0.0f;
	float si = 0.0f;
	float averageI;
	float averageD;
	int numI = 0;
	int numD = 0;
	int countNoSenseI = 0;
	int countNoSenseD = 0;
	int value = 0;
	for (unsigned int i = 0; i < sensors.size(); i++) {
		if (sensors.at(i).angle < angleRobot) {
			sd += sensors.at(i).distance;
			numD++;
			if (!sensors.at(i).instersected)
				countNoSenseD++;
		} else {
			si += sensors.at(i).distance;
			numI++;
			if (!sensors.at(i).instersected)
				countNoSenseI++;
		}
	}
	if (countNoSenseI == numI)
		countNoSenseI = 0;
	if (countNoSenseD == numD)
		countNoSenseD = 0;
	averageI = si / (float) (numI);
	 averageD = sd / (float) (numD);
	averageI = si;
	averageD = sd;

	if (averageI
			< rangeSensor * countNoSenseI
					+ thresholdSensor * (numI - countNoSenseI))
		value = (value << 1) + 1;
	else
		value = (value << 1) + 0;

	if (averageD
			< rangeSensor * countNoSenseD
					+ thresholdSensor * (numD - countNoSenseD))
		value = (value << 1) + 1;
	else
		value = (value << 1) + 0;

	value = value & 0xFFFFFFFF;
	return value;
}

simulator::MotionCommand generateOutputMotion(TypeMotion typeMotion,
		float distance, float angle) {
	simulator::MotionCommand output;
	switch (typeMotion) {
	case STOP:
		output.request.distance = 0.0f;
		output.request.angle = 0.0f;
		break;
	case FORWARD:
		output.request.distance = distance;
		output.request.angle = 0.0f;
		break;
	case BACKWARD:
		output.request.distance = -distance;
		output.request.angle = 0.0f;
		break;
	case LEFT:
		output.request.distance = 0.0f;
		output.request.angle = angle;
		break;
	case RIGHT:
		output.request.distance = 0.0f;
		output.request.angle = -angle;
		break;
	}
	return output;
}*/

float computeArea(Polygon polygon) {
	int i;
	float area = 0;
	for (i = 0; i < polygon.num_vertex; i++) {
		if (i < polygon.num_vertex - 1) {
			area = area + polygon.vertex[i].x * polygon.vertex[i + 1].y
					- polygon.vertex[i + 1].x * polygon.vertex[i].y;
		} else {
			area = area
					+ polygon.vertex[polygon.num_vertex - 1].x
							* polygon.vertex[0].y
					- polygon.vertex[0].x
							* polygon.vertex[polygon.num_vertex - 1].y;
		}
	}
	return area / 2;
}

std::vector<Vertex2> computeCentroids(std::vector<Polygon> polygons) {
	int i, j;
	float area;
	std::vector<Vertex2> centroids;
	for (i = 0; i < polygons.size(); i++) {
		Vertex2 centroid = Vertex2::Zero();
		area = computeArea(polygons[i]);
		for (j = 0; j < polygons[i].num_vertex; j++) {
			if (j < polygons[i].num_vertex - 1) {
				centroid.x =
						centroid.x
								+ (polygons[i].vertex[j].x
										+ polygons[i].vertex[j + 1].x)
										* (polygons[i].vertex[j].x
												* polygons[i].vertex[j + 1].y
												- polygons[i].vertex[j + 1].x
														* polygons[i].vertex[j].y);
				centroid.y =
						centroid.y
								+ (polygons[i].vertex[j].y
										+ polygons[i].vertex[j + 1].y)
										* (polygons[i].vertex[j].x
												* polygons[i].vertex[j + 1].y
												- polygons[i].vertex[j + 1].x
														* polygons[i].vertex[j].y);
			} else {
				centroid.x =
						centroid.x
								+ (polygons[i].vertex[polygons[i].num_vertex - 1].x
										+ polygons[i].vertex[0].x)
										* (polygons[i].vertex[polygons[i].num_vertex
												- 1].x * polygons[i].vertex[0].y
												- polygons[i].vertex[0].x
														* polygons[i].vertex[polygons[i].num_vertex
																- 1].y);
				centroid.y =
						centroid.y
								+ (polygons[i].vertex[polygons[i].num_vertex - 1].y
										+ polygons[i].vertex[0].y)
										* (polygons[i].vertex[polygons[i].num_vertex
												- 1].x * polygons[i].vertex[0].y
												- polygons[i].vertex[0].x
														* polygons[i].vertex[polygons[i].num_vertex
																- 1].y);
			}
		}
		centroid.x = centroid.x / (6 * area);
		centroid.y = centroid.y / (6 * area);
		centroids.push_back(centroid);
	}
	return centroids;
}

std::vector<biorobotics::Triangle> makeFloorEnvironment(
		biorobotics::Polygon * polygons, int num_polygons) {
	std::vector<biorobotics::Triangle> triangleFloor;
	float xmin, ymin, xmax, ymax;
	bool init = true;
	for (int i = 0; i < num_polygons; i++) {
		biorobotics::Polygon polygon = polygons[i];
		for (int j = 0; j < polygon.num_vertex; j++) {
			biorobotics::Vertex2 vertex = polygon.vertex[j];
			if (init) {
				xmin = vertex.x;
				xmax = vertex.x;
				ymin = vertex.y;
				ymax = vertex.y;
				init = false;
			} else {
				xmin = aisgl_min(xmin, vertex.x);
				xmax = aisgl_max(xmax, vertex.x);
				ymin = aisgl_min(ymin, vertex.y);
				ymax = aisgl_max(ymax, vertex.y);
			}
		}
	}

	Vertex3 v1, v2, v3, v4, v5, v6, v7, v8;
	v1.x = xmin;
	v1.y = ymin;
	v1.z = 0.0;
	v2.x = xmin;
	v2.y = ymax;
	v2.z = 0.0;
	v3.x = xmax;
	v3.y = ymax;
	v3.z = 0.0;
	v4.x = xmax;
	v4.y = ymin;
	v4.z = 0.0;
	v5.x = xmin;
	v5.y = ymin;
	v5.z = -0.3;
	v6.x = xmin;
	v6.y = ymax;
	v6.z = -0.3;
	v7.x = xmax;
	v7.y = ymax;
	v7.z = -0.3;
	v8.x = xmax;
	v8.y = ymin;
	v8.z = -0.3;

	Triangle t1(v3, v2, v1);
	Triangle t2(v1, v4, v3);

	Triangle t3(v1, v2, v6);
	Triangle t4(v1, v6, v5);

	Triangle t5(v1, v5, v8);
	Triangle t6(v1, v8, v4);

	Triangle t7(v4, v8, v3);
	Triangle t8(v8, v7, v3);

	Triangle t9(v2, v3, v6);
	Triangle t10(v3, v7, v6);

	Triangle t11(v5, v6, v7);
	Triangle t12(v5, v7, v8);

	triangleFloor.push_back(t1);
	triangleFloor.push_back(t2);
	triangleFloor.push_back(t3);
	triangleFloor.push_back(t4);
	triangleFloor.push_back(t5);
	triangleFloor.push_back(t6);
	triangleFloor.push_back(t7);
	triangleFloor.push_back(t8);
	triangleFloor.push_back(t9);
	triangleFloor.push_back(t10);
	triangleFloor.push_back(t11);
	triangleFloor.push_back(t12);

	return triangleFloor;
}

} /* namespace biorobotics */

#endif /* UTILSIMULATOR_H_ */

