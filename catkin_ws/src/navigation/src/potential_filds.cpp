#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <common/LaserScanUtil.h>
#include <common/GoalPotentialFieldsAction.h>
#include <navigation/PotentialFields.h>
#include <common/NavigationUtil.h>

#include "actionlib/server/simple_action_server.h"

LaserScanUtil laserUtil;
NavigationUtil navigationUtil;

float DELTA = 0.24;

class GoalPotentialFieldsAction {
public:
	GoalPotentialFieldsAction(std::string name) : action_name(name) {
		as = new actionlib::SimpleActionServer<common::GoalPotentialFieldsAction>(nh, name,
					boost::bind(&GoalPotentialFieldsAction::executeCallback, this, _1),
					false);
		as->start();

		algoPf = new PotentialFields(20, 1.2, 0.5, 20, 20);

		laserUtil.initRosConnection(&nh);
		navigationUtil.initRosConnection(&nh);

		tf_listener = new tf::TransformListener();
		tf_listener->waitForTransform("map", "base_link", ros::Time(0),
			ros::Duration(5.0));
		tf_listener->waitForTransform("odom", "base_link", ros::Time(0),
			ros::Duration(5.0));
		tf_listener->waitForTransform("map", "laser_link", ros::Time(0),
			ros::Duration(5.0));
		tf_listener->waitForTransform("odom", "laser_link", ros::Time(0),
			ros::Duration(5.0));
	}
	~GoalPotentialFieldsAction(){
		delete algoPf;
		delete tf_listener;
	}
	void executeCallback(const common::GoalPotentialFieldsGoalConstPtr msg) {
		std::cout << "New Goal Dist angle:" << msg->pose.x << "," << msg->pose.y
				<< "," << msg->pose.theta << std::endl;
		bool success = false, preempted = false;

		boost::posix_time::ptime prev =
			boost::posix_time::second_clock::local_time();
		boost::posix_time::ptime curr = prev;

		float goalX, goalY, goalTheta, timeout;

		goalX = msg->pose.x;
		goalY = msg->pose.y;
		goalTheta = msg->pose.theta;
		timeout = msg->timeout;

		ros::Rate rate(2);
		
		while(ros::ok() && ((curr - prev).total_milliseconds() < timeout || timeout == 0)){

			if (as->isPreemptRequested()){
    			ROS_INFO("%s: Preempted", " Action PotentialFields");
    			as->setPreempted();
    			preempted = true;
  			}

			tf::StampedTransform laserTransform;
			tf::StampedTransform robotTransform;
			tf_listener->lookupTransform("odom", "laser_link", ros::Time(0),
				laserTransform);
			tf_listener->lookupTransform("odom", "base_link", ros::Time(0),
				robotTransform);
			Vertex2 robotPos(robotTransform.getOrigin().x(), robotTransform.getOrigin().y());
			Vertex2 laserPos(laserTransform.getOrigin().x(), laserTransform.getOrigin().y());

			float errorX = goalX - robotTransform.getOrigin().x();
			float errorY = goalY - robotTransform.getOrigin().y();
			float error = sqrt(pow(errorX, 2) + pow(errorY, 2));
			if (error < 0.5) {
				navigationUtil.asyncMoveDistAngle(0, 0);
				success = true;
				break;
			}

			LaserScan * laserScan = laserUtil.getLaserScan();


			if(laserScan != 0){
				
				Vertex2 goalPos(msg->pose.x, msg->pose.y);
				biorobotics::Vertex2 totalForze = algoPf->computeTotalForzeWithSensors(laserPos,
					tf::getYaw(robotTransform.getRotation()), laserScan,goalPos);

				biorobotics::Vertex2 nextposition;
	            nextposition.x = robotPos.x
	                    - DELTA * totalForze.x / totalForze.norm();
	            nextposition.y = robotPos.y
	                    - DELTA * totalForze.y / totalForze.norm();

	             std::cout << "nextposition:" << nextposition.x << "," << nextposition.y << std::endl;
	            float distance = robotPos.sub(nextposition).norm();

	            float deltaX = nextposition.x - robotPos.x;
	            float deltaY = nextposition.y - robotPos.y;

	            float angleTarget = atan2(deltaY, deltaX);
	            if (angleTarget < 0.0f)
	                angleTarget = 2 * PI + angleTarget;

	            float turn = angleTarget - tf::getYaw(robotTransform.getRotation());
	            float advance = distance;

	            //navigationUtil.asyncMoveDistAngle(advance, turn);
	            //navigationUtil.syncMoveDistAngle(advance, turn, 0);
	            navigationUtil.asyncMovePose(nextposition.x, nextposition.y, turn);
			}

			rate.sleep();
			ros::spinOnce();
		}
		
		if(!preempted){
			result.success = success;
			as->setSucceeded(result);
		}
	}
private:
	std::string action_name;
	ros::NodeHandle nh;
	actionlib::SimpleActionServer<common::GoalPotentialFieldsAction> * as;
	common::GoalPotentialFieldsResult result;

	LaserScanUtil laserUtil;
	NavigationUtil navigationUtil;
	PotentialFields * algoPf;
	tf::TransformListener* tf_listener;
};

int main(int argc, char ** argv){

	ros::init(argc, argv, "potential_fields_node");

	ros::NodeHandle n;

	GoalPotentialFieldsAction goalPotentialFieldsAction("potential_fields_action");

	ros::spin();

	return 1;

}