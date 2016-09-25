/**
 * Definition.h
 * Fecha de creación: 12/02/2016, 10:21:08
 *
 * 2016 Universidad Nacional Autónoma de México - UNAM.
 * Instituto de Investigaciones en Matemáticas Aplicadas y en Sistemas - IIMAS.
 * Laboratorio de Bio-robótica.
 * Señales Imagenes y Ambientes Virtuales.
 */

#ifndef DEFINITION_H_
#define DEFINITION_H_

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstddef>
#include <vector>

#include <ros/ros.h>

#define PI 3.141592653589793238462643383280

namespace biorobotics {

enum ObjectType {
	WALL, OBSTACLE
};

typedef struct _Vertex2 {
	float x, y;
	_Vertex2() {
	}
	_Vertex2(float x, float y) {
		this->x = x;
		this->y = y;
	}
	static _Vertex2 Zero() {
		_Vertex2 vertex;
		vertex.x = 0.0;
		vertex.y = 0.0;
		return vertex;
	}
	_Vertex2 sub(_Vertex2 v) {
		_Vertex2 vertex;
		vertex.x = v.x - x;
		vertex.y = v.y - y;
		return vertex;
	}
	float norm() {
		return sqrt(pow(x, 2) + pow(y, 2));
	}
} Vertex2;

typedef struct _Vertex3 {
	float x, y, z;
	_Vertex3() {
	}
	_Vertex3(float x, float y, float z) {
		this->x = x;
		this->y = y;
		this->z = z;
	}
	static _Vertex3 Zero() {
		_Vertex3 vertex;
		vertex.x = 0.0;
		vertex.y = 0.0;
		vertex.z = 0.0;
		return vertex;
	}
	_Vertex3 sub(_Vertex3 v) {
		_Vertex3 vertex;
		vertex.x = v.x - x;
		vertex.y = v.y - y;
		vertex.z = v.z - z;
		return vertex;
	}
	float norm() {
		return sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
	}
} Vertex3;

typedef struct _Polygon {
	Vertex2 * vertex;
	int num_vertex = 0;
	ObjectType objectType;
} Polygon;

/* --- TODO This change to struct vertex verify --- */
typedef struct _Triangle {
	Vertex3 v1, v2, v3;
	ObjectType objectType;
	_Triangle(Vertex3 v1, Vertex3 v2, Vertex3 v3,
			ObjectType objectType) {
		this->v1 = v1;
		this->v2 = v2;
		this->v3 = v3;
		this->objectType = objectType;
	}
	_Triangle(Vertex3 v1, Vertex3 v2, Vertex3 v3) {
		this->v1 = v1;
		this->v2 = v2;
		this->v3 = v3;
	}
} Triangle;

/*typedef struct _Box {
	Eigen::Vector3d center;
	Eigen::Vector3d boxhalfsize;
} Box;*/
/* --- END This change to struct vertex verify --- */

typedef struct _AABB{
	_AABB(float minx, float maxx, float miny, float maxy, float minz, float maxz){
		this->minx = minx;
		this->miny = miny;
		this->minz = minz;
		this->maxx = maxx;
		this->maxy = maxy;
		this->maxz = maxz;
	}
	float minx;
	float maxx;
	float miny;
	float maxy;
	float minz;
	float maxz;
} AABB;

typedef struct _Segment {
	Vertex2 v1, v2;
	_Segment() {
	}
	_Segment(Vertex2 v1, Vertex2 v2) {
		this->v1 = v1;
		this->v2 = v2;
	}
} Segment;

typedef struct _Circle {
	Vertex2 center;
	float ratio;
	_Circle() {
	}
	_Circle(Vertex2 center, float ratio) {
		this->center = center;
		this->ratio = ratio;
	}
} Circle;

typedef struct _LaserScan{
	_LaserScan(){
	}
	_LaserScan(float angle_min, float angle_max, float angle_increment, float range_min, float range_max, 
		float num_scans, float * ranges){
		this->angle_min = angle_min;
		this->angle_max = angle_max;
		this->angle_increment = angle_increment;
		this->range_min = range_min;
		this->range_max = range_max;
		this->num_scans = num_scans;
		this->ranges = ranges;
	}
	~_LaserScan(){
		//delete []ranges;
	}
	float angle_min;
	float angle_max;
	float angle_increment;
	float range_min;
	float range_max;
	float num_scans;
	float * ranges = 0;
} LaserScan;

}

#endif /* DEFINITION_H_ */
