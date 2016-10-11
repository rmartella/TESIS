/**
 * UtilMap.h
 * Fecha de creación: 27/02/2016, 11:31:18
 *
 * Desarrollador Reynaldo Martell Avila
 * 2016 Universidad Nacional Autónoma de México - UNAM.
 * Instituto de Investigaciones en Matemáticas Aplicadas y en Sistemas - IIMAS.
 * Laboratorio de Bio-robótica.
 * Señales Imagenes y Ambientes Virtuales.
 */
#ifndef UTILMAP_H_
#define UTILMAP_H_

#include <visualization_msgs/Marker.h>

#include "utilSimulator.h"
#include "intersectionTest.h"

namespace biorobotics {

void computeParallelLines(Segment segment, Segment * segment1,
		Segment * segment2, float ratio);

std::vector<Polygon> scalePolygons(std::vector<Polygon> polygons,
		float robotRatio, float scale) {

	std::vector<Polygon> grownPolygons;
	std::vector<Vertex2> centroids = computeCentroids(polygons);

	for (int i = 0; i < polygons.size(); i++) {
		Polygon polygon = polygons[i];
		Polygon grownPolygon;
		Vertex2 * vertex = new Vertex2[polygon.num_vertex];
		for (int j = 0; j < polygon.num_vertex; j++) {
			Vertex2 v = Vertex2::Zero();
			v.x = scale * (polygon.vertex[j].x - centroids[i].x)
					+ centroids[i].x;
			v.y = scale * (polygon.vertex[j].y - centroids[i].y)
					+ centroids[i].y;
			vertex[j] = v;
		}
		grownPolygon.objectType = polygon.objectType;
		grownPolygon.vertex = vertex;
		grownPolygon.num_vertex = polygon.num_vertex;
		grownPolygons.push_back(grownPolygon);
	}
	return grownPolygons;
}

std::vector<Polygon> grownPolygons(std::vector<Polygon> polygons,
		float robotRatio) {

	std::vector<Polygon> grownPolygons;
	std::vector<Segment> edgesGrownPolygons;
	std::vector<int> offsetEdges;
	std::vector<int> sizeEdgesPolygons;
	int count = 0;

	for (int i = 0; i < polygons.size(); i++) {
		Polygon polygon = polygons[i];
		Polygon grownPolygon;
		for (int j = 0; j < polygon.num_vertex; j++) {
			Vertex2 v = Vertex2::Zero();
			Vertex2 v1 = polygon.vertex[j];
			Vertex2 v2;
			if (j < polygon.num_vertex - 1)
				v2 = polygon.vertex[j + 1];
			else
				v2 = polygon.vertex[0];
			Segment s(v1, v2);
			Segment s1;
			Segment s2;
			computeParallelLines(s, &s1, &s2, robotRatio);
			float determinante1 = getDeterminant(s.v2, s.v1, s1.v1);
			float determinante2 = getDeterminant(s.v2, s.v1, s2.v1);
			if (determinante1 > 0)
				edgesGrownPolygons.push_back(s1);
			else if (determinante2 > 0)
				edgesGrownPolygons.push_back(s2);
		}
		offsetEdges.push_back(count);
		sizeEdgesPolygons.push_back(polygon.num_vertex);
		count += polygon.num_vertex;
	}

	for (int i = 0; i < polygons.size(); i++) {
		Polygon polygon = polygons[i];
		Polygon grownPolygon;
		Vertex2 * vertex = new Vertex2[polygon.num_vertex];
		//std::vector<Segment> segmenVector = edgesGrownPolygons[offsetEdges[i]];
		for (int j = 0; j <= polygon.num_vertex - 1; j++) {
			Vertex2 v = Vertex2::Zero();
			//Segment seg1 = segmenVector[j];
			Segment seg1 = edgesGrownPolygons[offsetEdges[i] + j];
			Segment seg2;
			if (j == 0)
				seg2 = edgesGrownPolygons[offsetEdges[i] + sizeEdgesPolygons[i]
						- 1];
			else
				seg2 = edgesGrownPolygons[offsetEdges[i] + j - 1];
			Vertex2 inter = computeSegmentsIntersection(seg1, seg2);
			v.x = inter.x;
			v.y = inter.y;
			vertex[j] = v;
		}

		grownPolygon.objectType = polygon.objectType;
		grownPolygon.vertex = vertex;
		grownPolygon.num_vertex = polygon.num_vertex;
		grownPolygons.push_back(grownPolygon);
	}

	return grownPolygons;
}

Vertex2 * createVertexPointerFromPolygons(Polygon * polygons, int sizePolygons,
		int * sizeVertexMapPtr) {
	int totalSizeVertex = 0;
	int * offset = (int *) malloc(sizePolygons * sizeof(int));
	for (int i = 0; i < sizePolygons; i++) {
		offset[i] = totalSizeVertex;
		totalSizeVertex += polygons[i].num_vertex;
	}
	Vertex2 * vertexMap = (Vertex2 *) malloc(sizeof(Vertex2) * totalSizeVertex);
	for (int i = 0; i < sizePolygons; i++) {
		Polygon polygon = polygons[i];
		for (int j = 0; j < polygon.num_vertex; j++) {
			vertexMap[offset[i] + j] = polygon.vertex[j];
		}
	}
	*sizeVertexMapPtr = totalSizeVertex;
	return vertexMap;
}

bool ** computeMapTopologic(std::vector<Polygon> polygons,
		std::vector<Polygon> grownPolygons, float ratio,
		int * sizeAdyacencies) {

	int totalSizeVertex = 0;
	int * offset = (int *) malloc(polygons.size() * sizeof(int));
	for (int i = 0; i < polygons.size(); i++) {
		offset[i] = totalSizeVertex;
		totalSizeVertex += polygons[i].num_vertex;
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

	for (int h = 0; h < polygons.size(); h++) {
		Polygon polygon1 = grownPolygons[h];
		for (int i = h; i < polygons.size(); i++) {
			if (i != h) {
				Polygon polygon2 = grownPolygons[i];
				for (int j = 0; j < polygon1.num_vertex; j++) {
					Vertex2 vertex1 = polygon1.vertex[j];
					for (int k = 0; k < polygon2.num_vertex; k++) {
						Vertex2 vertex2 = polygon2.vertex[k];
						if (vertex1.x == vertex2.x && vertex1.y == vertex2.y) {
							continue;
						}
						Segment segment(vertex1, vertex2);
						Segment s1;
						Segment s2;
						computeParallelLines(segment, &s1, &s2, ratio);

						bool isIntersectBool = false;
						for (int l = 0; l < polygons.size(); l++) {
							Polygon polygonTest = polygons[l];
							for (int m = 0; m < polygonTest.num_vertex; m++) {
								Vertex2 verticeTest1 = polygonTest.vertex[m];
								Vertex2 verticeTest2;
								if (m < polygonTest.num_vertex - 1)
									verticeTest2 = polygonTest.vertex[m + 1];
								else
									verticeTest2 = polygonTest.vertex[0];
								Segment st;
								st.v1 = verticeTest1;
								st.v2 = verticeTest2;
								if (testSegmentIntersect(segment, st)
										|| testSegmentIntersect(s1, st)
										|| testSegmentIntersect(s2, st)) {
									isIntersectBool = true;
									break;
								}
							}
							if (isIntersectBool) {
								break;
							}
						}
						if (!isIntersectBool) {
							adyacencies[offset[h] + j][offset[i] + k] = 1;
							adyacencies[offset[i] + k][offset[h] + j] = 1;
						}
					}
				}
			} else {
				for (int j = 0; j < polygon1.num_vertex; j++) {
					Vertex2 vertex1 = polygon1.vertex[j];
					Vertex2 vertex2;
					if (j < polygon1.num_vertex - 1)
						vertex2 = polygon1.vertex[j + 1];
					else
						vertex2 = polygon1.vertex[0];

					Segment segment(vertex1, vertex2);
					bool isIntersectBool = false;
					for (int l = 0; l < polygons.size(); l++) {
						Polygon polygonTest = polygons[l];
						for (int m = 0; m < polygonTest.num_vertex; m++) {
							Vertex2 verticeTest1 = polygonTest.vertex[m];
							Vertex2 verticeTest2;
							if (m < polygonTest.num_vertex - 1)
								verticeTest2 = polygonTest.vertex[m + 1];
							else
								verticeTest2 = polygonTest.vertex[0];
							Segment st;
							st.v1 = verticeTest1;
							st.v2 = verticeTest2;
							if (testSegmentIntersect(segment, st)) {
								isIntersectBool = true;
								break;
							}
						}
						if (isIntersectBool)
							break;
					}
					if (!isIntersectBool) {
						if (j < polygon1.num_vertex - 1) {
							adyacencies[offset[h] + j][offset[h] + j + 1] = 1;
							adyacencies[offset[h] + j + 1][offset[h] + j] = 1;
						} else {
							adyacencies[offset[h] + j][offset[h]] = 1;
							adyacencies[offset[h]][offset[h] + j];
						}
					}
				}
			}
		}
	}
	delete offset;
	return adyacencies;
}

void computeParallelLines(Segment segment, Segment * segment1,
		Segment * segment2, float ratio) {
	float delX = segment.v2.x - segment.v1.x;
	float delY = segment.v2.y - segment.v1.y;
	if (delX == 0) {
		segment1->v1.x = segment.v1.x - ratio;
		segment1->v1.y = segment.v1.y;
		segment1->v2.x = segment.v2.x - ratio;
		segment1->v2.y = segment.v2.y;
		segment2->v1.x = segment.v1.x + ratio;
		segment2->v1.y = segment.v1.y;
		segment2->v2.x = segment.v2.x + ratio;
		segment2->v2.y = segment.v2.y;
	} else if (delY > -0.3 && delY < 0.3) {
		segment1->v1.x = segment.v1.x;
		segment1->v1.y = segment.v1.y - ratio;
		segment1->v2.x = segment.v2.x;
		segment1->v2.y = segment.v2.y - ratio;
		segment2->v1.x = segment.v1.x;
		segment2->v1.y = segment.v1.y + ratio;
		segment2->v2.x = segment.v2.x;
		segment2->v2.y = segment.v2.y + ratio;
	} else {
		float inverPendient = -delX / delY;

		float xaux1 = segment.v1.x - 10 * ratio;
		float xaux2 = segment.v1.x + 10 * ratio;
		float yaux1 = inverPendient * (xaux1 - segment.v1.x) + segment.v1.y;
		float yaux2 = inverPendient * (xaux2 - segment.v1.x) + segment.v1.y;
		float a = pow(xaux2 - xaux1, 2) + pow(yaux2 - yaux1, 2);
		float b = 2 * (xaux2 - xaux1) * (xaux1 - segment.v1.x)
				+ 2 * (yaux2 - yaux1) * (yaux1 - segment.v1.y);
		float c = pow(xaux1 - (float) segment.v1.x, 2)
				+ pow(yaux1 - (float) segment.v1.y, 2) - pow(ratio, 2);
		float t1 = (-b + sqrt(pow(b, 2) - 4 * a * c)) / (2 * a);
		float t2 = (-b - sqrt(pow(b, 2) - 4 * a * c)) / (2 * a);
		float x1int = (xaux2 - xaux1) * t1 + xaux1;
		float y1int = (yaux2 - yaux1) * t1 + yaux1;
		float x2int = (xaux2 - xaux1) * t2 + xaux1;
		float y2int = (yaux2 - yaux1) * t2 + yaux1;
		segment1->v1.x = x1int;
		segment1->v1.y = y1int;
		segment2->v1.x = x2int;
		segment2->v1.y = y2int;

		xaux1 = segment.v2.x - 10 * ratio;
		xaux2 = segment.v2.x + 10 * ratio;
		yaux1 = inverPendient * (xaux1 - segment.v2.x) + segment.v2.y;
		yaux2 = inverPendient * (xaux2 - segment.v2.x) + segment.v2.y;
		a = pow(xaux2 - xaux1, 2) + pow(yaux2 - yaux1, 2);
		b = 2 * (xaux2 - xaux1) * (xaux1 - segment.v2.x)
				+ 2 * (yaux2 - yaux1) * (yaux1 - segment.v2.y);
		c = pow(xaux1 - (float) segment.v2.x, 2)
				+ pow(yaux1 - (float) segment.v2.y, 2) - pow(ratio, 2);
		t1 = (-b + sqrt(pow(b, 2) - 4 * a * c)) / (2 * a);
		t2 = (-b - sqrt(pow(b, 2) - 4 * a * c)) / (2 * a);
		x1int = (xaux2 - xaux1) * t1 + xaux1;
		y1int = (yaux2 - yaux1) * t1 + yaux1;
		x2int = (xaux2 - xaux1) * t2 + xaux1;
		y2int = (yaux2 - yaux1) * t2 + yaux1;
		segment1->v2.x = x1int;
		segment1->v2.y = y1int;
		segment2->v2.x = x2int;
		segment2->v2.y = y2int;
	}
}

Vertex2 * addVertexInitEndToVertexMap(Vertex2 init, Vertex2 end,
		Vertex2 * vertexMap, int * sizeVertexMapPtr) {
	*sizeVertexMapPtr = *sizeVertexMapPtr + 2;
	vertexMap = (Vertex2 *) realloc(vertexMap,
			*sizeVertexMapPtr * sizeof(Vertex2));
	vertexMap[*sizeVertexMapPtr - 2] = init;
	vertexMap[*sizeVertexMapPtr - 1] = end;
	return vertexMap;
}

bool testToPathDirect(Vertex2 inicio, Vertex2 fin, Polygon * polygons, int num_polygons, float ratio) {
 	bool pathFree = true;

 	for (int i = 0; i < num_polygons && pathFree; i++) {
 		Polygon polygon = polygons[i];
 		for (int j = 0; j < polygon.num_vertex && pathFree; j++) {
 			Vertex2 vertex1 = polygon.vertex[j];
 			Vertex2 vertex2;
	 		if (j < polygon.num_vertex - 1)
	 			vertex2 = polygon.vertex[j + 1];
	 		else
	 			vertex2 = polygon.vertex[0];
	 		Segment sie(vertex1, vertex2);
			Segment s1;
			Segment s2;
			computeParallelLines(Segment(inicio, fin), &s1, &s2, ratio);
	 		//pathFree = !testSegmentIntersect(Segment(inicio, fin), Segment(vertex1, vertex2));
	 		if (testSegmentIntersect(Segment(inicio, fin), sie)
					|| testSegmentIntersect(s1, sie)
					|| testSegmentIntersect(s2, sie))
	 			pathFree = false;
 		}
 	}
 	return pathFree;
}

bool ** addNodesInitEndToMap(Vertex2 init, Vertex2 end, Polygon * polygons,
		int sizePolygons, Vertex2 * vertexMap, bool ** adyacencies,
		int * sizeAdjacenciesPtr, float ratio) {

	*sizeAdjacenciesPtr = *sizeAdjacenciesPtr + 2;
	adyacencies = (bool **) realloc(adyacencies,
			*sizeAdjacenciesPtr * sizeof(bool *));
	for (int i = 0; i < *sizeAdjacenciesPtr - 2; i++) {
		adyacencies[i] = (bool *) realloc(adyacencies[i],
				*sizeAdjacenciesPtr * sizeof(bool));
	}
	for (int i = *sizeAdjacenciesPtr - 2; i < *sizeAdjacenciesPtr; i++) {
		adyacencies[i] = (bool *) malloc(*sizeAdjacenciesPtr * sizeof(bool));
	}
	for (int i = *sizeAdjacenciesPtr - 2; i < *sizeAdjacenciesPtr; i++) {
		for (int j = 0; j < *sizeAdjacenciesPtr; j++) {
			adyacencies[i][j] = 0;
			adyacencies[j][i] = 0;
		}
	}

	for (int i = 0; i < *sizeAdjacenciesPtr - 2; i++) {
		Vertex2 vertexTargetBind = vertexMap[i];
		Segment sI1;
		Segment sI2;
		Segment sF1;
		Segment sF2;
		Segment sI(init, vertexTargetBind);
		Segment sF(end, vertexTargetBind);
		computeParallelLines(sI, &sI1, &sI2, ratio);
		computeParallelLines(sF, &sF1, &sF2, ratio);

		bool isIntersectInicioBool = false;
		bool isIntersectFinBool = false;

		for (int k = 0;
				k < sizePolygons
						&& (!isIntersectInicioBool || !isIntersectFinBool);
				k++) {
			for (int l = 0;
					l < polygons[k].num_vertex
							&& (!isIntersectInicioBool || !isIntersectFinBool);
					l++) {
				Vertex2 vertex1 = polygons[k].vertex[l];
				Vertex2 vertex2;
				if (l < polygons[k].num_vertex - 1)
					vertex2 = polygons[k].vertex[l + 1];
				else
					vertex2 = polygons[k].vertex[0];
				if (!isIntersectInicioBool
						&& (testSegmentIntersect(sI, Segment(vertex1, vertex2))
								|| testSegmentIntersect(sI1,
										Segment(vertex1, vertex2))
								|| testSegmentIntersect(sI2,
										Segment(vertex1, vertex2))))
					isIntersectInicioBool = true;
				if (!isIntersectFinBool
						&& (testSegmentIntersect(sF, Segment(vertex1, vertex2))
								|| testSegmentIntersect(sF1,
										Segment(vertex1, vertex2))
								|| testSegmentIntersect(sF2,
										Segment(vertex1, vertex2))))
					isIntersectFinBool = true;
			}
		}

		if (!isIntersectInicioBool) {
			adyacencies[i][*sizeAdjacenciesPtr - 2] = 1;
			adyacencies[*sizeAdjacenciesPtr - 2][i] = 1;
		}
		if (!isIntersectFinBool) {
			adyacencies[i][*sizeAdjacenciesPtr - 1] = 1;
			adyacencies[*sizeAdjacenciesPtr - 1][i] = 1;
		}

	}
	bool pathFree = testToPathDirect(init, end, polygons, sizePolygons, ratio);
	if (pathFree) {
		adyacencies[*sizeAdjacenciesPtr - 1][*sizeAdjacenciesPtr - 2] = 1;
	 	adyacencies[*sizeAdjacenciesPtr - 2][*sizeAdjacenciesPtr - 1] = 1;
	}
	return adyacencies;
}

} /* namespace biorobotics */

#endif /* UTILMAP_H_ */
