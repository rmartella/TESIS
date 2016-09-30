#include <ros/ros.h>

#include <fstream>

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

#include "common/PathPlanningUtil.h"
#include "common/NavigationUtil.h"

#include "common/utilViz.h"

#include "actionlib/server/simple_action_server.h"
#include "common/MotionPlannerSymAction.h"
#include "common/MotionPlannerGridAction.h"

std::map<std::string, std::vector<float> > loadKnownLocations(std::string path){
    std::cout << "Loading known locations from " << path << std::endl;
    std::vector<std::string> lines;
    std::ifstream file(path.c_str());
    std::string tempStr;
    std::map<std::string, std::vector<float> > locations;
    while(std::getline(file, tempStr))
        lines.push_back(tempStr);

    //Extraction of lines without comments
    for(size_t i=0; i< lines.size(); i++){
        size_t idx = lines[i].find("//");
        if(idx!= std::string::npos)
            lines[i] = lines[i].substr(0, idx);
    }

    locations.clear();
    float locX, locY, locAngle;
    bool parseSuccess;
    for(size_t i=0; i<lines.size(); i++){
        //std::cout << "MvnPln.->Parsing line: " << lines[i] << std::endl;
        std::vector<std::string> parts;
        std::vector<float> loc;
        boost::split(parts, lines[i], boost::is_any_of(" ,\t"), boost::token_compress_on);
        if(parts.size() < 3)
            continue;
        //std::cout << "MvnPln.->Parsing splitted line: " << lines[i] << std::endl;
        parseSuccess = true;
        std::stringstream ssX(parts[1]);
        if(!(ssX >> locX)) parseSuccess = false;
        std::stringstream ssY(parts[2]);
        if(!(ssY >> locY)) parseSuccess = false;
        loc.push_back(locX);
        loc.push_back(locY);
        if(parts.size() >= 4){
            std::stringstream ssAngle(parts[3]);
            if(!(ssAngle >> locAngle)) parseSuccess = false;
            loc.push_back(locAngle);
        }

        if(parseSuccess)
            locations[parts[0]] = loc;
    }
    std::cout << "Total number of known locations: " << locations.size() << std::endl;
    for(std::map<std::string, std::vector<float> >::iterator it=locations.begin(); it != locations.end(); it++){
        std::cout << "Location " << it->first << " " << it->second[0] << " " << it->second[1];
        if(it->second.size() > 2)
            std::cout << " " << it->second[2];
        std::cout << std::endl;
    }
    if(locations.size() < 1)
        std::cout << "WARNING: Cannot load known locations from file: " << path << ". There are no known locations." << std::endl;
    return locations;
}

class MotionPlannerSymAction {

    enum States{
        SM_INIT = 0,
        SM_GET_NEXT_POSITION = 1,
        SM_CALC_NEXT_MOTION = 3,
        SM_PREPARE_NEXT_GOAL = 4,
        SM_NAV_NEXT_GOAL = 5,
        SM_WAIT_REACHED_GOAL = 6,
        SM_CORRECT_ANGLE = 7,
        SM_REPLANNING = 8,
        SM_FINISH = 9
    };

public:
    MotionPlannerSymAction(std::string name, std::map<std::string, std::vector<float> > locations) 
        : action_name(name), locations(locations){
        as = new actionlib::SimpleActionServer<common::MotionPlannerSymAction>(nh, name,
                    boost::bind(&MotionPlannerSymAction::executeCallback, this, _1),
                    false);
        as->start();
        
        pathPlanningUtil.initRosConnection(&nh);
        navigationUtil.initRosConnection(&nh);
    }
    ~MotionPlannerSymAction(){
    }
    void executeCallback(const common::MotionPlannerSymGoalConstPtr msg) {
        bool success = false, preempted = false, correctAngle = false;

        std::cout << "New Goal Motion planner:" << msg->goalPose.x << "," << msg->goalPose.y
                << "," << msg->goalPose.theta << std::endl;

        float currX, currY, currTheta, goalX, goalY, goalTheta;

        float timeout = msg->timeout;

        boost::posix_time::ptime prev =
                boost::posix_time::second_clock::local_time();
        boost::posix_time::ptime curr = prev;

        if(msg->location.compare("") != 0){
            goalX = locations[msg->location][0];
            goalY = locations[msg->location][1];
            if(locations[msg->location].size() > 2){
                goalTheta = locations[msg->location][2];
                correctAngle = true;
            }
        }else{
            goalX = msg->goalPose.x;
            goalY = msg->goalPose.y;
            if(msg->correctAngle){
                goalTheta = msg->goalPose.theta;
                correctAngle = true;
            }
        }

        nav_msgs::Path path;
        float turn, nextX, nextY;
        int indexCurrPath = 0;

        float deltaX, deltaY, angleTarget;

        int state = SM_INIT;

        ros::Rate rate(30);

        while(ros::ok() && ((curr - prev).total_milliseconds() < timeout || timeout == 0) 
                && state != SM_FINISH){
            if (as->isPreemptRequested()){
                ROS_INFO("%s: Preempted", " Action MotionPlannerSymAction");
                as->setPreempted();
                preempted = true;
                break;
            }
            navigationUtil.getCurrPose(currX, currY, currTheta);

            switch(state){
                case SM_INIT:
                    path = pathPlanningUtil.planPathSymbMapDjsk(currX, currY, goalX, goalY, success);
                    if(success){
                        success = false;
                        state = SM_GET_NEXT_POSITION;
                    }
                    else
                        state = SM_FINISH;
                break;
                case SM_GET_NEXT_POSITION:
                    if(indexCurrPath < path.poses.size()){
                        nextX = path.poses[indexCurrPath].pose.position.x;
                        nextY = path.poses[indexCurrPath].pose.position.y;
                        state = SM_CALC_NEXT_MOTION;
                    }
                    else{

                        if(correctAngle)
                            state = SM_CORRECT_ANGLE;
                        else{
                            success = true;
                            state = SM_FINISH;
                        }
                    }
                break;
                case SM_CALC_NEXT_MOTION:

                    deltaX = nextX - currX;
                    deltaY = nextY - currY;

                    angleTarget = atan2(deltaY, deltaX);
                    if (angleTarget < 0.0f)
                        angleTarget = 2 * M_PI + angleTarget;

                    turn = angleTarget - currTheta;
                    state = SM_PREPARE_NEXT_GOAL;
                break;
                case SM_PREPARE_NEXT_GOAL:
                    navigationUtil.syncMoveDistAngle(0, turn, 10000);
                    state = SM_NAV_NEXT_GOAL;
                break;
                case SM_NAV_NEXT_GOAL:
                    navigationUtil.asyncPotentialFields(nextX, nextY);
                    state = SM_WAIT_REACHED_GOAL;
                break;
                case SM_WAIT_REACHED_GOAL:
                    if(navigationUtil.finishedCurrMotionPF()){
                        indexCurrPath++;
                        std::cout << "Go to the new position i:" << indexCurrPath << std::endl;
                        state = SM_GET_NEXT_POSITION;
                    }
                    else
                        state = SM_WAIT_REACHED_GOAL;
                break;
                case SM_CORRECT_ANGLE:
                    turn = goalTheta- currTheta;
                    if(turn > M_PI) 
                        turn -= 2*M_PI;
                    if(turn <= -M_PI) 
                        turn += 2*M_PI;
                    success = navigationUtil.syncMoveDistAngle(0, turn, 10000);
                    std::cout << "FINSH" << std::endl;
                    state = SM_FINISH;
                    navigationUtil.stopMotion();
                break;
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
    actionlib::SimpleActionServer<common::MotionPlannerSymAction> * as;
    common::MotionPlannerSymResult result;

    PathPlanningUtil pathPlanningUtil;
    NavigationUtil navigationUtil;

    std::map<std::string, std::vector<float> > locations;
};

class MotionPlannerGridAction {

    enum States{
        SM_INIT = 0,
        SM_GET_NEXT_POSITION = 1,
        SM_NAV_NEXT_GOAL = 2,
        SM_WAIT_REACHED_GOAL = 3,
        SM_CORRECT_ANGLE = 4,
        SM_REPLANNING = 5,
        SM_FINISH = 6
    };

public:
    MotionPlannerGridAction(std::string name, std::map<std::string, std::vector<float> > locations) 
        : action_name(name), locations(locations){
        as = new actionlib::SimpleActionServer<common::MotionPlannerGridAction>(nh, name,
                    boost::bind(&MotionPlannerGridAction::executeCallback, this, _1),
                    false);
        as->start();
        
        pathPlanningUtil.initRosConnection(&nh);
        navigationUtil.initRosConnection(&nh);
    }
    ~MotionPlannerGridAction(){
    }
    void executeCallback(const common::MotionPlannerGridGoalConstPtr msg) {
        bool success = false, preempted = false, correctAngle = false;

        std::cout << "New Goal Motion planner:" << msg->goalPose.x << "," << msg->goalPose.y
                << "," << msg->goalPose.theta << std::endl;

        float currX, currY, currTheta, goalX, goalY, goalTheta;

        float timeout = msg->timeout;

        boost::posix_time::ptime prev =
                boost::posix_time::second_clock::local_time();
        boost::posix_time::ptime curr = prev;

        if(msg->location.compare("") != 0){
            goalX = locations[msg->location][0];
            goalY = locations[msg->location][1];
            if(locations[msg->location].size() > 2){
                goalTheta = locations[msg->location][2];
                correctAngle = true;
            }
        }else{
            goalX = msg->goalPose.x;
            goalY = msg->goalPose.y;
            if(msg->correctAngle){
                goalTheta = msg->goalPose.theta;
                correctAngle = true;
            }
        }

        nav_msgs::Path path;
        float turn, nextX, nextY;
        int indexCurrPath = 0;

        float deltaX, deltaY, angleTarget;

        int state = SM_INIT;

        ros::Rate rate(30);

        while(ros::ok() && ((curr - prev).total_milliseconds() < timeout || timeout == 0) 
                && state != SM_FINISH){
            if (as->isPreemptRequested()){
                ROS_INFO("%s: Preempted", " Action MotionPlannerGridAction");
                as->setPreempted();
                preempted = true;
                break;
            }
            navigationUtil.getCurrPose(currX, currY, currTheta);

            switch(state){
                case SM_INIT:
                    path = pathPlanningUtil.planPathGridMap(currX, currY, goalX, goalY, success);
                    if(success){
                        success = false;
                        state = SM_GET_NEXT_POSITION;
                    }
                    else
                        state = SM_FINISH;
                break;
                case SM_GET_NEXT_POSITION:
                    if(indexCurrPath < path.poses.size()){
                        nextX = path.poses[indexCurrPath].pose.position.x;
                        nextY = path.poses[indexCurrPath].pose.position.y;
                        state = SM_NAV_NEXT_GOAL;
                    }
                    else{

                        if(correctAngle)
                            state = SM_CORRECT_ANGLE;
                        else{
                            success = true;
                            state = SM_FINISH;
                        }
                    }
                break;
                case SM_NAV_NEXT_GOAL:
                    navigationUtil.asyncPotentialFields(nextX, nextY);
                    //navigationUtil.asyncMovePose(nextX, nextY, 0, false);
                    state = SM_WAIT_REACHED_GOAL;
                break;
                case SM_WAIT_REACHED_GOAL:
                    if(navigationUtil.finishedCurrMotionPF()){
                    //if(navigationUtil.finishedCurrMotionPose()){
                        indexCurrPath++;
                        std::cout << "Go to the new position i:" << indexCurrPath << std::endl;
                        state = SM_GET_NEXT_POSITION;
                    }
                    else
                        state = SM_WAIT_REACHED_GOAL;
                break;
                case SM_CORRECT_ANGLE:
                    turn = goalTheta- currTheta;
                    if(turn > M_PI) 
                        turn -= 2*M_PI;
                    if(turn <= -M_PI) 
                        turn += 2*M_PI;
                    success = navigationUtil.syncMoveDistAngle(0, turn, 10000);
                    std::cout << "FINSH" << std::endl;
                    state = SM_FINISH;
                    navigationUtil.stopMotion();
                break;
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
    actionlib::SimpleActionServer<common::MotionPlannerGridAction> * as;
    common::MotionPlannerGridResult result;

    PathPlanningUtil pathPlanningUtil;
    NavigationUtil navigationUtil;

    std::map<std::string, std::vector<float> > locations;
};


int main(int argc, char ** argv){

	ros::init(argc, argv, "motion_planner_node");

	ros::NodeHandle n;

    std::string locationsFilePath = "";
    for(int i=0; i < argc; i++){
        std::string strParam(argv[i]);
        std::cout << "strParam:" << strParam << std::endl;
        if(strParam.compare("-f") == 0)
            locationsFilePath = argv[++i];
    }

    std::cout << "locationsFilePath:" << locationsFilePath << std::endl;

	std::map<std::string, std::vector<float> > locations = loadKnownLocations(locationsFilePath);

	MotionPlannerSymAction actionSym("sym_motion_planner_action", locations);
    MotionPlannerGridAction actionGtid("grid_motion_planner_action", locations);

    ros::Publisher loc_marker_pub = n.advertise<visualization_msgs::Marker>("loc_marker", 10);

    ros::Rate rate(15);

    while(ros::ok()){

        biorobotics::sendToVizLocationMarker(locations, &loc_marker_pub);

        rate.sleep();
        ros::spinOnce();
    }

    return 1;

}