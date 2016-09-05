/**
 * PointClickSubscriber.h
 * Fecha de creaci�n: 24/04/2016, 0:52:31
 *
 * Desarrollador Reynaldo Martell Avila
 * 2016 Universidad Nacional Aut�noma de M�xico - UNAM.
 * Instituto de Investigaciones en Matem�ticas Aplicadas y en Sistemas - IIMAS.
 * Laboratorio de Bio-rob�tica.
 * Se�ales Imagenes y Ambientes Virtuales.
 */
#ifndef SRC_POINTCLICKSUBSCRIBER_H_
#define SRC_POINTCLICKSUBSCRIBER_H_

#include <ros/ros.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point32.h>

class PointClickSubscriber {
public:
	PointClickSubscriber(ros::NodeHandle * n);
	virtual ~PointClickSubscriber();

	void executeCallback(geometry_msgs::PointStamped msg);

	geometry_msgs::Point32 getPoint() {
		return point;
	}

	void setPoint(geometry_msgs::Point32 point) {
		this->point = point;
	}

private:
	geometry_msgs::Point32 point;

protected:
	ros::Subscriber subscriber;
};

#endif /* SRC_POINTCLICKSUBSCRIBER_H_ */
