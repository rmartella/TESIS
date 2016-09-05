/**
 * laser_scan_publisher.cpp
 * Fecha de creación: 24/04/2016, 23:09:08
 *
 * Desarrollador Reynaldo Martell Avila
 * 2016 Universidad Nacional Autónoma de México - UNAM.
 * Instituto de Investigaciones en Matemáticas Aplicadas y en Sistemas - IIMAS.
 * Laboratorio de Bio-robótica.
 * Señales Imagenes y Ambientes Virtuales.
 */

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <common/EnvironmentUtil.h>
//#include <common/RobotPose.h>
#include <common/definition.h>
#include <common/intersectionTest.h>
 #include "tf/transform_listener.h"

float angle_min = -1.57;
float range_angle_sensor = 3.14;
int num_sensors = 1000;
float range_min = 0.0;
float range_max = 8.0;

int main(int argc, char ** argv) {

	ros::init(argc, argv, "lase_simul_scan");

	ros::NodeHandle n;
	ros::Rate rate(30);

	ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("scan", 1);

	EnvironmentUtil envu(&n);
	//RobotPose robotPose;
	tf::TransformListener * tf_listener = new tf::TransformListener();
	tf_listener->waitForTransform("map", "base_link", ros::Time(0), ros::Duration(5.0));
    tf_listener->waitForTransform("odom", "base_link", ros::Time(0), ros::Duration(5.0));

	biorobotics::Polygon * polygons_ptr = nullptr;
	int num_polygons = 0;

	double laser_frequency = 40;

	polygons_ptr = envu.convertGeometryMsgToPolygons(envu.call(),
				polygons_ptr, &num_polygons);

	while (ros::ok()) {

		float inc_angle;
		if (num_sensors == 1)
			inc_angle = range_angle_sensor;
		else
			inc_angle = (range_angle_sensor) / (num_sensors - 1);

		sensor_msgs::LaserScan scan;
		scan.header.stamp = ros::Time::now();
		scan.header.frame_id = "laser_link";
		scan.angle_min = angle_min;
		scan.angle_max = angle_min + range_angle_sensor;
		scan.angle_increment = inc_angle;
		//scan.time_increment = (1 / laser_frequency) / (num_sensors);
		scan.range_min = range_min;
		scan.range_max = range_max;

		scan.ranges.resize(num_sensors);
		//scan.intensities.resize(num_sensors);

		tf::StampedTransform transform;
    	tf_listener->lookupTransform("odom", "laser_link", ros::Time(0), transform);
    	float currX = transform.getOrigin().x();
		float currY = transform.getOrigin().y();
		float currTheta = tf::getYaw(transform.getRotation());

		for (int i = 0; i < num_sensors; i++) {
			scan.ranges[i] = scan.range_max + 1.0;

			float angleSensor = currTheta + i * inc_angle
					+ angle_min;
			float startX = currX + range_min * cos(angleSensor);
			float startY = currY + range_min * sin(angleSensor);
			float endX = currX + range_max * cos(angleSensor);
			float endy = currY + range_max * sin(angleSensor);

			float min = -1;
			bool laserint = false;
			for (int j = 0; j < num_polygons; j++) {
				biorobotics::Polygon polygon = polygons_ptr[j];
				for (int k = 0; k < polygon.num_vertex; k++) {
					biorobotics::Vertex2 v1 = polygon.vertex[k];
					biorobotics::Vertex2 v2;
					if (k < polygon.num_vertex - 1)
						v2 = polygon.vertex[k + 1];
					else
						v2 = polygon.vertex[0];
					bool testIntersect = biorobotics::testSegmentIntersect(
							biorobotics::Segment(
									biorobotics::Vertex2(startX, startY),
									biorobotics::Vertex2(endX, endy)),
							biorobotics::Segment(v1, v2));
					if (testIntersect) {
						laserint = true;
						biorobotics::Vertex2 vint =
								biorobotics::computeSegmentsIntersection(
										biorobotics::Segment(
												biorobotics::Vertex2(startX,
														startY),
												biorobotics::Vertex2(endX,
														endy)),
										biorobotics::Segment(v1, v2));
						float d =
								biorobotics::Vertex2(startX, startY).sub(vint).norm();
						if (min < 0)
							min = d;
						else if (d < min)
							min = d;
					}
				}
			}
			if(laserint)
				scan.ranges[i] = min;
			else
				scan.ranges[i] = range_max + 1;
		}

		scan_pub.publish(scan);

		ros::spinOnce();
		rate.sleep();
	}

}
