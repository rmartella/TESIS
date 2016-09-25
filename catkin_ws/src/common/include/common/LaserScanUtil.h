#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>

#include <common/definition.h>

class LaserScanUtil {
public:
	LaserScanUtil();
	LaserScanUtil(ros::NodeHandle * n);
	virtual ~LaserScanUtil();

	void initRosConnection(ros::NodeHandle * n);

	void executeCallback(sensor_msgs::LaserScan msg);

	biorobotics::LaserScan * getLaserScan(){
		return this->laserScan;
	}

private:
	biorobotics::LaserScan * laserScan;

protected:
	ros::Subscriber subscriber;
};