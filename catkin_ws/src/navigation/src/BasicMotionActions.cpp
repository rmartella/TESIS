#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "std_msgs/Float32MultiArray.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"

#include "actionlib/server/simple_action_server.h"

#include "navigation/LowLevelControl.h"

#include "common/GoalDistAction.h"
#include "common/GoalDistAngleAction.h"
#include "common/GoalPoseAction.h"
#include "common/GoalPathAction.h"

#include "common/intersectionTest.h"
#include "common/EnvironmentUtil.h"

#include <iostream>

enum ActionType{
	DISTANCE,
	DISTANCE_ANGLE,
	POSE,
	PATH
};

class BasicMotion {
public:
	void initRosConnection(ros::NodeHandle *n);
	void doGoalMotion(float goalX, float goalY, float goalTheta,
			bool correctAngle, bool moveBackwards, bool moveLateral,
			float timeOut, bool &success, bool& preempted,  biorobotics::Polygon * polygons,
			int num_polygons, ActionType actionType);
	void doAngleMotion(float goalTheta, float timeOut, bool &success, bool& preempted,
			biorobotics::Polygon * polygons, int num_polygons, ActionType actionType);
	tf::TransformListener* getTfListener() {
		return this->tf_listener;
	}


	actionlib::SimpleActionServer<common::GoalDistAction> * asd;
	actionlib::SimpleActionServer<common::GoalDistAngleAction> * asda;
	actionlib::SimpleActionServer<common::GoalPoseAction> * aspo;
	actionlib::SimpleActionServer<common::GoalPathAction> * aspa;
protected:
	tf::TransformListener* tf_listener;
	ros::Publisher pubSpeeds;
	ros::Publisher pubCmdVel;
	ros::Publisher pubPose;
	LowLevelControl control;
	ros::Publisher pub;
	std_msgs::Float32MultiArray pose;

};

void BasicMotion::initRosConnection(ros::NodeHandle *n) {
	pubSpeeds = n->advertise<std_msgs::Float32MultiArray>(
			"/hardware/mobile_base/speeds", 1);
	pubCmdVel = n->advertise<geometry_msgs::Twist>(
			"/hardware/mobile_base/cmd_vel", 1);
	pubPose = n->advertise<std_msgs::Float32MultiArray>(
			"/hardware/mobile_base/set_pose", 1);
	tf_listener = new tf::TransformListener();
	tf_listener->waitForTransform("map", "base_link", ros::Time(0),
			ros::Duration(5.0));
	tf_listener->waitForTransform("odom", "base_link", ros::Time(0),
			ros::Duration(5.0));
	pub = n->advertise<visualization_msgs::Marker>("collision_markers", 1);
	control.SetRobotParams(0.48);

	pose.data.push_back(0);
	pose.data.push_back(0);
	pose.data.push_back(0);
}

void BasicMotion::doGoalMotion(float goalX, float goalY, float goalTheta,
		bool correctAngle, bool moveBackwards, bool moveLateral, float timeout,
		bool& success, bool& preempted, biorobotics::Polygon * polygons, int num_polygons, ActionType actionType) {
	ros::Rate rate(30);
	bool correctFinalAngle = false;
	tf::StampedTransform transform;
	float currentX, currentY, currentTheta;
	float errorAngle;

	std_msgs::Float32MultiArray speeds;
	speeds.data.push_back(0);
	speeds.data.push_back(0);
	geometry_msgs::Twist twist;

	boost::posix_time::ptime prev =
			boost::posix_time::second_clock::local_time();
	boost::posix_time::ptime curr = prev;
	bool motionFinished = false;
	preempted = false;

	while (ros::ok()
			&& ((curr - prev).total_milliseconds() < timeout || timeout == 0)
			&& !motionFinished && !preempted) {

		tf_listener->lookupTransform("odom", "base_link", ros::Time(0),
					transform);
		currentX = transform.getOrigin().x();
		currentY = transform.getOrigin().y();
		currentTheta = tf::getYaw(transform.getRotation());

		bool testCollision = testAABBWithPolygons(currentX, currentY,
					currentTheta, 0.44, 0.44, polygons, num_polygons, &pub);
		if (testCollision) {
			std::cout << " ----- testCollision -----" << testCollision
					<< std::endl;
			pubPose.publish(pose);
			motionFinished = false;
			break;
		}

		pose.data[0] = currentX;
		pose.data[1] = currentY;
		pose.data[2] = currentTheta;

		if (!correctFinalAngle) {
			
			float errorX = goalX - currentX;
			float errorY = goalY - currentY;
			float error = sqrt(pow(errorX, 2) + pow(errorY, 2));
			if (error < 0.035) {
				speeds.data[0] = 0;
				speeds.data[1] = 0;
				pubSpeeds.publish(speeds);
				if (correctAngle)
					correctFinalAngle = true;
				else
					motionFinished = true;
			} else {
				if (!moveLateral) {
					control.CalculateSpeeds(currentX, currentY, currentTheta,
							goalX, goalY, speeds.data[0], speeds.data[1],
							moveBackwards);
					pubSpeeds.publish(speeds);
				} else {
					control.CalculateSpeedsLateral(currentX, currentY,
							currentTheta, goalX, goalY, twist.linear.y,
							twist.angular.z, moveBackwards);
					pubCmdVel.publish(twist);
				}
			}
		}
		if (correctFinalAngle) {
			
			errorAngle = goalTheta - currentTheta;
			if (errorAngle > M_PI)
				errorAngle -= 2 * M_PI;
			if (errorAngle <= -M_PI)
				errorAngle += 2 * M_PI;

			if (fabs(errorAngle) < 0.05) {
				speeds.data[0] = 0;
				speeds.data[1] = 0;
				pubSpeeds.publish(speeds);
				motionFinished = true;
			} else {
				control.CalculateSpeeds(currentTheta, goalTheta, speeds.data[0],
						speeds.data[1]);
				pubSpeeds.publish(speeds);
			}
		}
		rate.sleep();
		curr = boost::posix_time::second_clock::local_time();
		ros::spinOnce();

		switch(actionType){
			case DISTANCE:
				if (asd->isPreemptRequested()){
        			ROS_INFO("%s: Preempted", " Action Distance");
        			asd->setPreempted();
        			preempted = true;
      			}
				break;
			case DISTANCE_ANGLE:
				if (asda->isPreemptRequested()){
        			ROS_INFO("%s: Preempted", " Action Distance Angle");
        			asda->setPreempted();
        			preempted = true;
      			}
				break;
			case POSE:
				if (aspo->isPreemptRequested()){
        			ROS_INFO("%s: Preempted", " Action Pose");
        			aspo->setPreempted();
        			preempted = true;
      			}
				break;
			case PATH:
				if (aspa->isPreemptRequested()){
        			ROS_INFO("%s: Preempted", " Action Path");
        			aspa->setPreempted();
        			preempted = true;
      			}
				break;
		}
	}
	if (!motionFinished || preempted) {
		speeds.data[0] = 0;
		speeds.data[1] = 0;
		pubSpeeds.publish(speeds);
	}
	success = motionFinished & !preempted;
}

void BasicMotion::doAngleMotion(float goalTheta, float timeout, bool& success, bool& preempted, 
		biorobotics::Polygon * polygons, int num_polygons, ActionType actionType) {
	ros::Rate rate(30);
	tf::StampedTransform transform;
	float currentX, currentY, currentTheta;
	float errorAngle;

	std_msgs::Float32MultiArray speeds;
	speeds.data.push_back(0);
	speeds.data.push_back(0);
	geometry_msgs::Twist twist;

	boost::posix_time::ptime prev =
			boost::posix_time::second_clock::local_time();
	boost::posix_time::ptime curr = prev;
	bool motionFinished = false;
	preempted = false;

	while (ros::ok()
			&& ((curr - prev).total_milliseconds() < timeout || timeout == 0)
			&& !motionFinished && !preempted) {
		tf_listener->lookupTransform("odom", "base_link", ros::Time(0),
				transform);
		currentX = transform.getOrigin().x();
		currentY = transform.getOrigin().y();
		currentTheta = tf::getYaw(transform.getRotation());
		bool testCollision = testAABBWithPolygons(currentX, currentY,
				currentTheta, 0.44, 0.44, polygons, num_polygons, &pub);
		if (testCollision) {
			std::cout << " ----- testCollision -----" << testCollision
					<< std::endl;
			pubPose.publish(pose);
			motionFinished = false;
			break;
		}
		pose.data[0] = currentX;
		pose.data[1] = currentY;
		pose.data[2] = currentTheta;

		errorAngle = goalTheta - currentTheta;
		if (errorAngle > M_PI)
			errorAngle -= 2 * M_PI;
		if (errorAngle <= -M_PI)
			errorAngle += 2 * M_PI;

		if (fabs(errorAngle) < 0.04) {
			speeds.data[0] = 0;
			speeds.data[1] = 0;
			pubSpeeds.publish(speeds);
			motionFinished = true;
		} else {
			control.CalculateSpeeds(currentTheta, goalTheta, speeds.data[0],
					speeds.data[1]);
			pubSpeeds.publish(speeds);
		}
		rate.sleep();
		curr = boost::posix_time::second_clock::local_time();
		ros::spinOnce();
		switch(actionType){
			case DISTANCE:
				if (asd->isPreemptRequested()){
        			ROS_INFO("%s: Preempted", " Action Distance");
        			asd->setPreempted();
        			preempted = true;
      			}
				break;
			case DISTANCE_ANGLE:
				if (asda->isPreemptRequested()){
        			ROS_INFO("%s: Preempted", " Action Distance Angle");
        			asda->setPreempted();
        			preempted = true;
      			}
				break;
			case POSE:
				if (aspo->isPreemptRequested()){
        			ROS_INFO("%s: Preempted", " Action Pose");
        			aspo->setPreempted();
        			preempted = true;
      			}
				break;
			case PATH:
				if (aspa->isPreemptRequested()){
        			ROS_INFO("%s: Preempted", " Action Path");
        			aspa->setPreempted();
        			preempted = true;
      			}
				break;
		}
	}
	if (!motionFinished || preempted) {
		speeds.data[0] = 0;
		speeds.data[1] = 0;
		pubSpeeds.publish(speeds);
	}
	success = motionFinished & !preempted;
}

class GoalDistAction {
public:
	GoalDistAction(std::string name, BasicMotion *bm) : action_name(name), bm(bm), polygons_ptr(
					0), num_polygons(0){
		as = new actionlib::SimpleActionServer<common::GoalDistAction>(nh, name,
					boost::bind(&GoalDistAction::executeCallback, this, _1),
					false);
		as->start();
		bm->asd = as;
		envu.initRosConnection(&nh);
		polygons_ptr = 0;
	}
	~GoalDistAction(){
		delete as;
	}
	void executeCallback(const common::GoalDistGoalConstPtr msg) {
		std::cout << "New Goal Dist:" << msg->dist << std::endl;
		bool success, preempted;
		tf::StampedTransform transform;
		tf::TransformListener* tf_listener = bm->getTfListener();
		tf_listener->lookupTransform("odom", "base_link", ros::Time(0),
				transform);
		float currentX = transform.getOrigin().x();
		float currentY = transform.getOrigin().y();
		float currentTheta = tf::getYaw(transform.getRotation());

		float dist = msg->dist;
		float moveLateral = msg->moveLateral;
		float timeout = msg->timeout;

		float goalX, goalY, goalTheta;

		if (!moveLateral) {
			goalX = currentX + dist * cos(currentTheta);
			goalY = currentY + dist * sin(currentTheta);
			if (dist > 0)
				goalTheta = atan2(goalY - currentY, goalX - currentX);
			else
				goalTheta = currentTheta;
		} else {
			goalX = currentX + dist * cos(currentTheta + M_PI / 2);
			goalY = currentY + dist * sin(currentTheta + M_PI / 2);
			goalTheta = currentTheta;
		}

		if(polygons_ptr == 0)
			polygons_ptr = envu.convertGeometryMsgToPolygons(envu.call(),
					polygons_ptr, &num_polygons);

		bm->doGoalMotion(goalX, goalY, goalTheta, false,
				dist < 0 ? true : false, moveLateral, timeout, success, preempted,
				polygons_ptr, num_polygons, DISTANCE);


		if(!preempted){
			result.success = success;
			as->setSucceeded(result);
		}
	}
private:
	std::string action_name;
	ros::NodeHandle nh;
	actionlib::SimpleActionServer<common::GoalDistAction> * as;
	common::GoalDistResult result;
	BasicMotion * bm;
	EnvironmentUtil envu;
	biorobotics::Polygon * polygons_ptr;
	int num_polygons;
};

class GoalDistAngleAction {
public:
	GoalDistAngleAction(std::string name, BasicMotion * bm) : action_name(name), bm(bm), polygons_ptr(
					0), num_polygons(0) {
		as = new actionlib::SimpleActionServer<common::GoalDistAngleAction>(nh, name,
					boost::bind(&GoalDistAngleAction::executeCallback, this,
							_1), false);
		as->start();
		bm->asda = as;
		envu.initRosConnection(&nh);
		polygons_ptr = 0;
	}
	void executeCallback(const common::GoalDistAngleGoalConstPtr msg) {
		std::cout << "New Goal Dist angle:" << msg->dist << "," << msg->angle
				<< std::endl;
		bool success, preempted;
		tf::StampedTransform transform;
		tf::TransformListener* tf_listener = bm->getTfListener();
		tf_listener->lookupTransform("odom", "base_link", ros::Time(0),
				transform);
		float currentX = transform.getOrigin().x();
		float currentY = transform.getOrigin().y();
		//tf::Quaternion q = transform.getRotation();
		//float currentTheta = atan2((float)q.z(), (float)q.w()) * 2;
		float currentTheta = tf::getYaw(transform.getRotation());

		float goalX, goalY, goalTheta;

		goalTheta = currentTheta + msg->angle;
		if (goalTheta > 2 * M_PI)
			goalTheta -= 4 * M_PI;
		else if (goalTheta < -2 * M_PI)
			goalTheta += 4 * M_PI;

		if(polygons_ptr == 0)
			polygons_ptr = envu.convertGeometryMsgToPolygons(envu.call(),
					polygons_ptr, &num_polygons);

		boost::posix_time::ptime prev =
				boost::posix_time::second_clock::local_time();
		boost::posix_time::ptime curr;
		bm->doAngleMotion(goalTheta, msg->timeout, success, preempted,  polygons_ptr,
				num_polygons, DISTANCE_ANGLE);

		if (success) {
			tf_listener->lookupTransform("odom", "base_link", ros::Time(0),
					transform);
			currentX = transform.getOrigin().x();
			currentY = transform.getOrigin().y();
			currentTheta = tf::getYaw(transform.getRotation());
			goalX = currentX + msg->dist * cos(currentTheta);
			goalY = currentY + msg->dist * sin(currentTheta);
			goalTheta = currentTheta;
			curr = boost::posix_time::second_clock::local_time();
			bm->doGoalMotion(goalX, goalY, goalTheta, false,
					msg->dist < 0 ? true : false, false,
					msg->timeout == 0 ?
							0 :
							msg->timeout - (curr - prev).total_milliseconds(),
					success, preempted,polygons_ptr, num_polygons, DISTANCE_ANGLE);
		}

		if(!preempted){
			result.success = success;
			as->setSucceeded(result);
		}
	}
private:
	std::string action_name;
	ros::NodeHandle nh;
	actionlib::SimpleActionServer<common::GoalDistAngleAction> * as;
	common::GoalDistAngleResult result;
	BasicMotion * bm;
	EnvironmentUtil envu;
	biorobotics::Polygon * polygons_ptr;
	int num_polygons;
};

class GoalPoseAction {
public:
	GoalPoseAction(std::string name, BasicMotion * bm) : action_name(name), bm(bm) {
		as = new actionlib::SimpleActionServer<common::GoalPoseAction>(nh, name,
					boost::bind(&GoalPoseAction::executeCallback, this, _1),
					false);
		as->start();
		bm->aspo = as;
		envu.initRosConnection(&nh);
		polygons_ptr = 0;
	}
	void executeCallback(const common::GoalPoseGoalConstPtr msg) {
		std::cout << "New Goal Dist angle:" << msg->pose.x << "," << msg->pose.y
				<< "," << msg->pose.theta << std::endl;
		bool success, preempted;

		if(polygons_ptr == 0)
			polygons_ptr = envu.convertGeometryMsgToPolygons(envu.call(),
					polygons_ptr, &num_polygons);

		bm->doGoalMotion(msg->pose.x, msg->pose.y, msg->pose.theta, msg->correctAngle, false,
				false, msg->timeout, success, preempted, polygons_ptr, num_polygons, POSE);

		if(!preempted){
			result.success = success;
			as->setSucceeded(result);
		}
	}
private:
	std::string action_name;
	ros::NodeHandle nh;
	actionlib::SimpleActionServer<common::GoalPoseAction> * as;
	common::GoalPoseResult result;
	BasicMotion * bm;
	EnvironmentUtil envu;
	biorobotics::Polygon * polygons_ptr;
	int num_polygons;
};

class GoalPathAction {
public:
	GoalPathAction(std::string name, BasicMotion * bm) : action_name(name), bm(bm) {
		as = new actionlib::SimpleActionServer<common::GoalPathAction>(nh, name,
					boost::bind(&GoalPathAction::executeCallback, this, _1),
					false);
		as->start();
		bm->aspa = as;
		envu.initRosConnection(&nh);
		polygons_ptr = 0;
	}
	void executeCallback(const common::GoalPathGoalConstPtr msg) {
		std::cout << "New Goal Patn:" << std::endl;
		nav_msgs::Path path = msg->path;
		int indexCurrPath = 0;
		bool success, preempted;

		if(polygons_ptr == 0)
			polygons_ptr = envu.convertGeometryMsgToPolygons(envu.call(),
					polygons_ptr, &num_polygons);

		if (path.poses.size() > 0) {
			float timeout = msg->timeout;
			boost::posix_time::ptime prev =
					boost::posix_time::second_clock::local_time();
			boost::posix_time::ptime curr = prev;
			ros::Rate rate(30);
			do {
				geometry_msgs::PoseStamped poseStmp = path.poses[indexCurrPath];
				bm->doGoalMotion(poseStmp.pose.position.x,
						poseStmp.pose.position.y, 0, false, false, false,
						timeout == 0 ?
								0 :
								timeout - (curr - prev).total_milliseconds(),
						success, preempted, polygons_ptr, num_polygons, PATH);
				if (success)
					indexCurrPath++;
				rate.sleep();
				curr = boost::posix_time::second_clock::local_time();
			} while (ros::ok() && indexCurrPath < path.poses.size()
					&& ((curr - prev).total_milliseconds() < timeout
							|| timeout == 0) && success);
		}
		if(!preempted){
			result.success = success;
			as->setSucceeded(result);
		}
	}
private:
	std::string action_name;
	ros::NodeHandle nh;
	actionlib::SimpleActionServer<common::GoalPathAction> * as;
	common::GoalPathResult result;
	BasicMotion * bm;
	EnvironmentUtil envu;
	biorobotics::Polygon * polygons_ptr;
	int num_polygons;
};

int main(int argc, char ** argv) {
	ros::init(argc, argv, "basic_motion");
	ros::NodeHandle n;

	BasicMotion basicMotion;
	basicMotion.initRosConnection(&n);

	GoalDistAction goalDistAction("goal_dist_action", &basicMotion);
	GoalDistAngleAction goalDistAngleAction("goal_dist_angle_action",
			&basicMotion);
	GoalPoseAction goalPoseAction("goal_pose_action", &basicMotion);
	GoalPathAction goalPathAction("goal_path_action", &basicMotion);

	ros::Rate rate(30);
	while(ros::ok()){
		rate.sleep();
		ros::spinOnce();
	}
}
