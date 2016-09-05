/**
 * EnvironmentClient.h
 * Fecha de creación: 23/04/2016, 18:10:51
 *
 * Desarrollador Reynaldo Martell Avila
 * 2016 Universidad Nacional Autónoma de México - UNAM.
 * Instituto de Investigaciones en Matemáticas Aplicadas y en Sistemas - IIMAS.
 * Laboratorio de Bio-robótica.
 * Señales Imagenes y Ambientes Virtuales.
 */
#ifndef SRC_ENVIRONMENTUTIL_H_
#define SRC_ENVIRONMENTUTIL_H_

#include <ros/ros.h>
#include <geometry_msgs/Polygon.h>
#include <common/Environment.h>
#include <common/definition.h>

class EnvironmentUtil {
public:
	EnvironmentUtil();
	EnvironmentUtil(ros::NodeHandle * n);
	virtual ~EnvironmentUtil();

	void initRosConnection(ros::NodeHandle * n);
	std::vector<geometry_msgs::Polygon> call();
	biorobotics::Polygon * convertGeometryMsgToPolygons(
			std::vector<geometry_msgs::Polygon> polygonsMsg,
			biorobotics::Polygon * polygons_ptr, int * num_polygons_ptr);

protected:
	ros::ServiceClient * client;
};

#endif /* SRC_ENVIRONMENTUTIL_H_ */
