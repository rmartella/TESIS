
#include <common/LaserScanUtil.h>

LaserScanUtil::LaserScanUtil(){
	laserScan = nullptr;
}

LaserScanUtil::LaserScanUtil(ros::NodeHandle * n){
	laserScan = nullptr;
	subscriber = n->subscribe("/hardware/scan", 1,
			&LaserScanUtil::executeCallback, this);
}

LaserScanUtil::~LaserScanUtil(){
}

void LaserScanUtil::initRosConnection(ros::NodeHandle * n){
	subscriber = n->subscribe("/hardware/scan", 1,
			&LaserScanUtil::executeCallback, this);
}

void LaserScanUtil::executeCallback(sensor_msgs::LaserScan msg){
	int num_scans = msg.ranges.size();
	if(laserScan != nullptr)
		delete laserScan;

	float * ranges_ptr = (float *) malloc(sizeof(float) * num_scans);

	for(int i = 0; i < num_scans; i++)
		ranges_ptr[i] = msg.ranges[i];

	laserScan = new biorobotics::LaserScan(msg.angle_min, msg.angle_max, msg.angle_increment, msg.range_min, msg.range_max, 
		num_scans, ranges_ptr);
}