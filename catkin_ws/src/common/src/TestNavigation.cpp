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
	//nu.asyncMoveDist(dis, false);
	//success = nu.syncMoveDist(1.0, false, 8000);
	//nu.asyncMoveDistAngle(dis, angle);
	//success = nu.syncMoveDistAngle(dis, angle, 6000);
	nu.asyncMovePose(3.95, -3.13, 0.0, 0);
	//success = nu.syncMovePose(0.0, 0.0, 1.5708, 6000);
	/*nu.asyncPotentialFields(-2.5, 8.0);
	success = nu.syncPotentialFields(-2.5, 8.0, 3000);*/
	std::cout << "success:" << success << std::endl;

	return 1;

}