
#include <common/LaserScanUtil.h>

LaserScanUtil::LaserScanUtil(){
	laserScan = nullptr;
}

LaserScanUtil::LaserScanUtil(ros::NodeHandle * n){
	laserScan = nullptr;
	subscriber = n->subscribe("scan", 1,
			&LaserScanUtil::executeCallback, this);
}

LaserScanUtil::~LaserScanUtil(){
}

void LaserScanUtil::initRosConnection(ros::NodeHandle * n){
	subscriber = n->subscribe("scan", 1,
			&LaserScanUtil::executeCallback, this);
}

void LaserScanUtil::executeCallback(sensor_msgs::LaserScan msg){
	int num_scans = msg.ranges.size();
	if(laserScan != nullptr)
		delete laserScan;

	std::vector<float> ranges;
	ranges.reserve(num_scans);
	float * ranges_ptr;

	std::copy(std::begin(msg.ranges), std::end(msg.ranges), std::begin(ranges));

	ranges_ptr = ranges.data();

	laserScan = new biorobotics::LaserScan(msg.angle_min, msg.angle_max, msg.angle_increment, msg.range_min, msg.range_max, 
		num_scans, ranges_ptr);
}