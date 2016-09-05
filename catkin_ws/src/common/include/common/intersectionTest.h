/**
 * intersectionTest.h
 * Fecha de creación: 12/02/2016, 10:21:08
 *
 * 2016 Universidad Nacional Autónoma de México - UNAM.
 * Instituto de Investigaciones en Matemáticas Aplicadas y en Sistemas - IIMAS.
 * Laboratorio de Bio-robótica.
 * Señales Imagenes y Ambientes Virtuales.
 */

#ifndef INCLUDE_SIMULATOR_INTERSECTIONTEST_H_
#define INCLUDE_SIMULATOR_INTERSECTIONTEST_H_

#include "definition.h"
#include "utilSimulator.h"
#include <vector>
#include <math.h>

 #include <Eigen/Dense>

#define EPSILON 0.000001
#define FLT_MAX 1000.00

#define AXISTEST_X01(a, b, fa, fb) p0 = a*v0(1,0) - b*v0(2,0); p2 = a*v2(1,0) - b*v2(2,0); if(p0<p2) {min=p0; max=p2;} else {min=p2; max=p0;} rad = fa * boxhalfsize(1,0) + fb * boxhalfsize(2,0); if(min>rad || max<-rad) return 0;
#define AXISTEST_X2(a, b, fa, fb) p0 = a*v0(1,0) - b*v0(2,0); p1 = a*v1(1,0) - b*v1(2,0); if(p0<p1) {min=p0; max=p1;} else {min=p1; max=p0;} rad = fa * boxhalfsize(1,0) + fb * boxhalfsize(2,0); if(min>rad || max<-rad) return 0;

#define AXISTEST_Y02(a, b, fa, fb) p0 = -a*v0(0,0) + b*v0(2,0); p2 = -a*v2(0,0) + b*v2(2,0); if(p0<p2) {min=p0; max=p2;} else {min=p2; max=p0;}rad = fa * boxhalfsize(0,0) + fb * boxhalfsize(2,0); if(min>rad || max<-rad) return 0;
#define AXISTEST_Y1(a, b, fa, fb) p0 = -a*v0(0, 0) + b*v0(2,0); p1 = -a*v1(0, 0) + b*v1(2,0); if(p0<p1) {min=p0; max=p1;} else {min=p1; max=p0;} rad = fa * boxhalfsize(0, 0) + fb * boxhalfsize(2,0); if(min>rad || max<-rad) return 0;

#define AXISTEST_Z12(a, b, fa, fb) p1 = a*v1(0,0) - b*v1(1,0); p2 = a*v2(0,0) - b*v2(1,0); if(p2<p1) {min=p2; max=p1;} else {min=p1; max=p2;} rad = fa * boxhalfsize(0,0) + fb * boxhalfsize(1,0); if(min>rad || max<-rad) return 0;
#define AXISTEST_Z0(a, b, fa, fb) p0 = a*v0(0,0) - b*v0(1,0); p1 = a*v1(0,0) - b*v1(1,0); if(p0<p1) {min=p0; max=p1;} else {min=p1; max=p0;} rad = fa * boxhalfsize(0,0) + fb * boxhalfsize(1,0); if(min>rad || max<-rad) return 0;

#define FINDMINMAX(x0,x1,x2,min,max) min = max = x0; if(x1<min) min=x1; if(x1>max) max=x1; if(x2<min) min=x2; if(x2>max) max=x2;

namespace biorobotics {

bool testSegmentIntersect(Segment segment1, Segment segment2) {
	float determinante1 = getDeterminant(segment1.v2, segment1.v1, segment2.v1);
	float determinante2 = getDeterminant(segment1.v2, segment1.v1, segment2.v2);
	if ((determinante1 < 0 && determinante2 > 0)
			|| (determinante1 > 0 && determinante2 < 0)) {
		determinante1 = getDeterminant(segment2.v2, segment2.v1, segment1.v1);
		determinante2 = getDeterminant(segment2.v2, segment2.v1, segment1.v2);
		if ((determinante1 < 0 && determinante2 > 0)
				|| (determinante1 > 0 && determinante2 < 0))
			return true;
		else
			return false;
	} else if (determinante1 == 0 || determinante2 == 0)
		return true;
	else
		return false;
}

Vertex2 computeSegmentsIntersection(Segment s1, Segment s2) {
	//float A1 = s1.getPoint2().getY() - s1.getPoint1().getY();
	float A1 = s1.v2.y - s1.v1.y;
	//float B1 = s1.getPoint1().getX() - s1.getPoint2().getX();
	float B1 = s1.v1.x - s1.v2.x;
	//float C1 = A1 * s1.getPoint1().getX() + B1 * s1.getPoint1().getY();
	float C1 = A1 * s1.v1.x + B1 * s1.v1.y;
	//float A2 = s2.getPoint2().getY() - s2.getPoint1().getY();
	float A2 = s2.v2.y - s2.v1.y;
	//float B2 = s2.getPoint1().getX() - s2.getPoint2().getX();
	float B2 = s2.v1.x - s2.v2.x;
	//float C2 = A2 * s2.getPoint1().getX() + B2 * s2.getPoint1().getY();
	float C2 = A2 * s2.v1.x + B2 * s2.v1.y;
	double det = A1 * B2 - A2 * B1;
	double x = (B2 * C1 - B1 * C2) / det;
	double y = (A1 * C2 - A2 * C1) / det;

	return Vertex2(x, y);
}

/*float computeDistanceEuclideanPointToLine(Eigen::Vector3d point,
 Segment segment) {
 Eigen::Vector3d u;
 Eigen::Vector3d p0;
 u(0, 0) = segment.v2.x - segment.v1.x;
 u(1, 0) = segment.v2.y - segment.v1.y;
 u(2, 0) = 0.0;
 p0(0, 0) = segment.v1.x;
 p0(1, 0) = segment.v1.y;
 p0(2, 0) = 0.0;
 Eigen::Vector3d ba = point - p0;
 return ba.cross(u).norm() / u.norm();
 }*/

bool isInRangeProyection(Vertex2 vertex, Segment segment) {
	float xinf, xsup, yinf, ysup;
	if (segment.v1.x < segment.v2.x) {
		xinf = segment.v1.x;
		xsup = segment.v2.x;
	} else {
		xsup = segment.v1.x;
		xinf = segment.v2.x;
	}
	if (segment.v1.y < segment.v2.y) {
		yinf = segment.v1.y;
		ysup = segment.v2.y;
	} else {
		ysup = segment.v1.y;
		yinf = segment.v2.y;
	}
	if (vertex.x >= xinf && vertex.x <= xsup && vertex.y >= yinf
			&& vertex.y <= ysup)
		return true;
	else
		return false;
}

bool segmentCircleIntersect(Circle circle, Segment segment) {
	bool result = false;
	float dx = segment.v2.x - segment.v1.x;
	float dy = segment.v2.y - segment.v1.y;
	float h = circle.center.x;
	float k = circle.center.y;
	float r = circle.ratio;
	float x0 = segment.v1.x;
	float y0 = segment.v1.y;

	float a = pow(dx, 2) + pow(dy, 2);
	float b = 2 * dx * (x0 - h) + 2 * dy * (y0 - k);
	float c = pow(x0 - (float) h, 2) + pow(y0 - (float) k, 2) - pow(r, 2);

	float disc = pow(b, 2) - 4 * a * c;

	if (disc == 0 || disc > 0) {
		float t1 = (-b + sqrt(disc)) / (2 * a);
		float t2 = (-b - sqrt(disc)) / (2 * a);

		float x1int = dx * t1 + x0;
		float y1int = dy * t1 + y0;
		float x2int = dx * t2 + x0;
		float y2int = dy * t2 + y0;

		if (disc == 0) {
			//std::cout << "Tangent" << std::endl;
			result = isInRangeProyection(Vertex2(x1int, y1int), segment);
		} else if (disc > 0) {
			//std::cout << "Intersection" << std::endl;
			result = isInRangeProyection(Vertex2(x1int, y1int), segment);
			if (!result)
				result = isInRangeProyection(Vertex2(x2int, y2int), segment);
		}
	} else if (disc < 0) {
		//std::cout << "Not intersect" << std::endl;
	}
	return result;
}

bool testCircleWithPolygons(float posx, float posy, float radius,
		Polygon * polygons, int num_polygons) {

	bool testcollision = false;
	Circle shapeRobot(Vertex2(posx, posy), radius);

	for (int i = 0; i < num_polygons && !testcollision; i++) {
		Polygon polygon = polygons[i];
		for (int j = 0; j < polygon.num_vertex && !testcollision; j++) {
			if (j < polygon.num_vertex - 1) {
				testcollision = segmentCircleIntersect(shapeRobot,
						Segment(
								Segment(polygon.vertex[j],
										polygon.vertex[j + 1])));
			} else {
				testcollision = segmentCircleIntersect(shapeRobot,
						Segment(Segment(polygon.vertex[j], polygon.vertex[0])));
			}
		}
	}

	return testcollision;
}

bool testSLABPlane(float p, float d, float min, float max, float &tmin){
	float tmax = FLT_MAX;
	if(abs(d) < EPSILON){
		if(p < min || p > max) 
			return false;
	}
	else{
		float ood = 1.0f / d;
		float t1 = (min - p) * ood;
		float t2 = (max - p) * ood;
		if(t1 > t2){
			float aux = t1;
			t1 = t2;
			t2 = aux;
		}
		if (t1 > tmin)
			tmin = t1;
		if (t2 > tmax)
			tmax = t2;
		if (tmin > tmax)
			return false;
	}
	return true;
}

bool intersectRayAABB(Vertex2 p, Vertex2 d, AABB a){
	std::cout << "P(" << p.x << "," << p.y << ")" << std::endl;
	std::cout << "D(" << d.x << "," << d.y << ")" << std::endl;
	std::cout << "AAAB(" << a.minx << "," << a.maxx << "," << a.miny << "," << a.maxy << ")" << std::endl;
	bool test = false;
	float tmin;
	Vertex2 q;
	tmin = 0.0f;
	test = testSLABPlane(p.x, d.x, a.minx, a.maxx, tmin);
	if(!test)
		test = testSLABPlane(p.y, d.y, a.miny, a.maxy, tmin);
	q.x = p.x + d.x * tmin;
	q.y = p.y + d.y * tmin;
	return test;
}

bool testAABBWithPolygons(float posx, float posy, float theta, float width, float height, Polygon * polygons, int num_polygons){
	bool testcollision = false;
	AABB shapeRobot(posx - width/2, posx + width/2, posy - height/2, posy + height/2, 0, 0);
	Eigen::Matrix4d rotRobot = Eigen::Affine3d(Eigen::AngleAxisd(theta, Eigen::Vector3d(0, 0, 1))).matrix();
	Eigen::Matrix3d rotation = rotRobot.block<3, 3>(0, 0);
	Eigen::Matrix3d inv = rotation.inverse();

	for (int i = 0; i < num_polygons && !testcollision; i++) {
		Polygon polygon = polygons[i];
		for (int j = 0; j < polygon.num_vertex && !testcollision; j++) {
			Eigen::Vector3d pe = Eigen::Vector3d::Zero();
			Eigen::Vector3d de = Eigen::Vector3d::Zero();
			if (j < polygon.num_vertex - 1) {
				pe(0,0) = polygon.vertex[j].x;
				pe(1,0) = polygon.vertex[j].y;
				de(0,0) = polygon.vertex[j + 1].x - polygon.vertex[j].x;
				de(1,0) = polygon.vertex[j + 1].y - polygon.vertex[j].y;
			} else {
				pe(0,0) = polygon.vertex[j].x;
				pe(1,0) = polygon.vertex[j].y;
				de(0,0) = polygon.vertex[0].x - polygon.vertex[j].x;
				de(1,0) = polygon.vertex[0].y - polygon.vertex[j].y;
			}
			Eigen::Vector3d peo = inv * pe;
			Eigen::Vector3d deo = inv * de;
			Vertex2 p(peo(0,0), peo(1,0));
			Vertex2 d(deo(0,0), deo(1,0));
			testcollision = intersectRayAABB(p, d, shapeRobot);
		}
	}
	return testcollision;
}

}

#endif /* INCLUDE_SIMULATOR_INTERSECTIONTEST_H_ */
