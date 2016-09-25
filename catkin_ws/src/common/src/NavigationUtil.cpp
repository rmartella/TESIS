
#include "common/NavigationUtil.h"

NavigationUtil::~NavigationUtil(){
	delete acMovDis;
	delete acMovDisAngle;
	delete acMovPose;
	delete acMovPath;
	delete acMovPotFields;
}

void NavigationUtil::initRosConnection(ros::NodeHandle * n){
	acMovDis = new actionlib::SimpleActionClient<common::GoalDistAction>("goal_dist_action", true);
	acMovDisAngle = new actionlib::SimpleActionClient<common::GoalDistAngleAction>("goal_dist_angle_action", true);
	acMovPose = new actionlib::SimpleActionClient<common::GoalPoseAction>("goal_pose_action", true);
	acMovPath = new actionlib::SimpleActionClient<common::GoalPathAction>("goal_path_action", true);
	acMovPotFields = new actionlib::SimpleActionClient<common::GoalPotentialFieldsAction>("potential_fields_action", true);
}

void NavigationUtil::asyncMoveDist(float dist, bool moveLateral){
	std::cout << "Move distance:" << dist << "," << moveLateral << std::endl;
	acMovDis->waitForServer();
	common::GoalDistGoal msg;
	msg.dist = dist;
	msg.moveLateral = moveLateral;
	msg.timeout = 0;
	acMovDis->sendGoal(msg);
}

bool NavigationUtil::syncMoveDist(float dist, bool moveLateral, int timeout){
	std::cout << "Move distance:" << dist << "," << moveLateral << std::endl;
	acMovDis->waitForServer();
	common::GoalDistGoal msg;
	msg.dist = dist;
	msg.moveLateral = moveLateral;
	msg.timeout = timeout;
	acMovDis->sendGoal(msg);

	bool finished_before_timeout = acMovDis->waitForResult(ros::Duration(0.0));

    actionlib::SimpleClientGoalState state = acMovDis->getState();
    if(state ==  actionlib::SimpleClientGoalState::SUCCEEDED)
    	return acMovDis->getResult()->success;
    else
    	return false;	
}

void NavigationUtil::asyncMoveDistAngle(float dist, float theta){
	std::cout << "Move distance and angle:" << dist << "," << theta << std::endl;
	acMovDisAngle->waitForServer();
	common::GoalDistAngleGoal msg;
	msg.dist = dist;
	msg.angle = theta;
	msg.timeout = 0;
	acMovDisAngle->sendGoal(msg);
}

bool NavigationUtil::syncMoveDistAngle(float dist, float theta, int timeout){
	std::cout << "Move distance and angle:" << dist << "," << theta << std::endl;
	acMovDisAngle->waitForServer();
	common::GoalDistAngleGoal msg;
	msg.dist = dist;
	msg.angle = theta;
	msg.timeout = timeout;
	acMovDisAngle->sendGoal(msg);

	bool finished_before_timeout = acMovDisAngle->waitForResult(ros::Duration(0.0));

    actionlib::SimpleClientGoalState state = acMovDisAngle->getState();
    if(state ==  actionlib::SimpleClientGoalState::SUCCEEDED){
    	common::GoalDistAngleResultConstPtr result = acMovDisAngle->getResult();
    	return result->success;
    }
    else
    	return false;
}

void NavigationUtil::asyncMovePose(float goalX, float goalY, float goalTheta){
	std::cout << "Move to pose:" << goalX << "," << goalY << "," << goalTheta << std::endl;
	acMovPose->waitForServer();
	common::GoalPoseGoal msg;
	msg.pose.x = goalX;
	msg.pose.y = goalY;
	msg.pose.theta = goalTheta;
	msg.timeout = 0;
	acMovPose->sendGoal(msg);
}

bool NavigationUtil::syncMovePose(float goalX, float goalY, float goalTheta, int timeout){
	std::cout << "Move to pose:" << goalX << "," << goalY << "," << goalTheta << std::endl;
	acMovPose->waitForServer();
	common::GoalPoseGoal msg;
	msg.pose.x = goalX;
	msg.pose.y = goalY;
	msg.pose.theta = goalTheta;
	msg.timeout = timeout;
	acMovPose->sendGoal(msg);

	bool finished_before_timeout = acMovPose->waitForResult(ros::Duration(0.0));

    actionlib::SimpleClientGoalState state = acMovPose->getState();
    if(state ==  actionlib::SimpleClientGoalState::SUCCEEDED)
    	return acMovPose->getResult()->success;
    else
    	return false;
}

void NavigationUtil::asyncMovePath(nav_msgs::Path path){
	std::cout << "Move to path:" << std::endl;
	acMovPath->waitForServer();
	common::GoalPathGoal msg;
	msg.path = path;
	msg.timeout = 0;
	acMovPath->sendGoal(msg);
}

bool NavigationUtil::syncMovePath(nav_msgs::Path path, int timeout){
	std::cout << "Move to path:" << std::endl;
	acMovPath->waitForServer();
	common::GoalPathGoal msg;
	msg.path = path;
	msg.timeout = timeout;
	acMovPath->sendGoal(msg);

	bool finished_before_timeout = acMovPath->waitForResult(ros::Duration(0.0));

    actionlib::SimpleClientGoalState state = acMovPath->getState();
    if(state ==  actionlib::SimpleClientGoalState::SUCCEEDED)
    	return acMovPath->getResult()->success;
    else
    	return false;
}

void NavigationUtil::asyncPotentialFields(float goalX, float goalY){
	std::cout << "Move to pose:" << goalX << "," << goalY << std::endl;
	acMovPotFields->waitForServer();
	common::GoalPotentialFieldsGoal msg;
	msg.pose.x = goalX;
	msg.pose.y = goalY;
	msg.timeout = 0;
	acMovPotFields->sendGoal(msg);
}

bool NavigationUtil::syncPotentialFields(float goalX, float goalY, int timeout){
	std::cout << "Move to pose:" << goalX << "," << goalY << std::endl;
	acMovPotFields->waitForServer();
	common::GoalPotentialFieldsGoal msg;
	msg.pose.x = goalX;
	msg.pose.y = goalY;
	msg.timeout = timeout;
	acMovPotFields->sendGoal(msg);

	bool finished_before_timeout = acMovPotFields->waitForResult(ros::Duration(0.0));

    actionlib::SimpleClientGoalState state = acMovPotFields->getState();
    if(state ==  actionlib::SimpleClientGoalState::SUCCEEDED)
    	return acMovPotFields->getResult()->success;
    else
    	return false;
}