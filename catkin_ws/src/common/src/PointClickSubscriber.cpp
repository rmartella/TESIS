/**
 * PointClickSubscriber.cpp
 * Fecha de creaci�n: 24/04/2016, 0:52:31
 *
 * Desarrollador Reynaldo Martell Avila
 * 2016 Universidad Nacional Aut�noma de M�xico - UNAM.
 * Instituto de Investigaciones en Matem�ticas Aplicadas y en Sistemas - IIMAS.
 * Laboratorio de Bio-rob�tica.
 * Se�ales Imagenes y Ambientes Virtuales.
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

