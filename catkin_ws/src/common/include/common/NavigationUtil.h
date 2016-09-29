#include "ros/ros.h"
#include "tf/transform_listener.h"

#include "common/GoalDistAction.h"
#include "common/GoalDistAngleAction.h"
#include "common/GoalPathAction.h"
#include "common/GoalPoseAction.h"
#include "common/GoalPotentialFieldsAction.h"

#include "geometry_msgs/Pose2D.h"

#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"

class NavigationUtil{
public:

	~NavigationUtil();

	void initRosConnection(ros::NodeHandle * n);
	void getCurrPose(float &x, float &y, float &theta);
	void asyncMoveDist(float dist, bool moveLateral);
	bool syncMoveDist(float dist, bool moveLateral, int timeout);
	void asyncMoveDistAngle(float dist, float theta);
	bool syncMoveDistAngle(float dist, float theta, int timeout);
	void asyncMovePose(float goalX, float goalY, float goalTheta, bool correctAngle);
	bool syncMovePose(float goalX, float goalY, float goalTheta, bool correctAngle, int timeout);
	void asyncMovePath(nav_msgs::Path path);
	bool syncMovePath(nav_msgs::Path path, int timeout);
	void asyncPotentialFields(float goalX, float goalY);
	bool syncPotentialFields(float goalX, float goalY, int timeout);

	void stopMotion();

	bool finishedCurrMotionDist();
	bool finishedCurrMotionDistAngle();
	bool finishedCurrMotionPose();
	bool finishedCurrMotionPath();
	bool finishedCurrMotionPF();

protected:
	actionlib::SimpleActionClient<common::GoalDistAction> * acMovDis;
	actionlib::SimpleActionClient<common::GoalDistAngleAction> * acMovDisAngle;
	actionlib::SimpleActionClient<common::GoalPoseAction> * acMovPose;
	actionlib::SimpleActionClient<common::GoalPathAction> * acMovPath;
	actionlib::SimpleActionClient<common::GoalPotentialFieldsAction> * acMovPotFields;

	tf::TransformListener* tf_listener;
};