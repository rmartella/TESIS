/**
 * PointClickSubscriber.cpp
 * Fecha de creación: 24/04/2016, 0:52:31
 *
 * Desarrollador Reynaldo Martell Avila
 * 2016 Universidad Nacional Autónoma de México - UNAM.
 * Instituto de Investigaciones en Matemáticas Aplicadas y en Sistemas - IIMAS.
 * Laboratorio de Bio-robótica.
 * Señales Imagenes y Ambientes Virtuales.
 */
#include "common/PointClickSubscriber.h"

PointClickSubscriber::PointClickSubscriber(ros::NodeHandle * n) {
	subscriber = n->subscribe("clicked_point", 1,
			&PointClickSubscriber::executeCallback, this);
}

PointClickSubscriber::~PointClickSubscriber() {
	// TODO Auto-generated destructor stub
}
void PointClickSubscriber::executeCallback(geometry_msgs::PointStamped msg) {
	std::cout << "Execute PointClickSubscriber" << std::endl;
	point.x = msg.point.x;
	point.y = msg.point.y;
	point.z = msg.point.z;
}

