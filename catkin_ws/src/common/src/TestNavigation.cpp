#include "ros/ros.h"

#include "common/NavigationUtil.h"

#include <iostream>

int main(int argc, char ** argv){
	ros::init(argc, argv, "test_navigation");
	ros::NodeHandle n;

	NavigationUtil nu;
	nu.initRosConnection(&n);

	float dis = atof(argv[1]);
	float angle = atof(argv[2]);

	bool success;
	//nu.asyncMoveDist(3.00, true);
	//success = nu.syncMoveDist(-3.0, true, 8000);
	//nu.asyncMoveDistAngle(dis, angle);
	//success = nu.syncMoveDistAngle(-dis, -angle, 6000);
	nu.asyncMovePose(0.0, 0.0, 1.5708);
	success = nu.syncMovePose(0.0, 0.0, 1.5708, 6000);
	std::cout << "success:" << success << std::endl;

	return 1;

}