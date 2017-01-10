#include <ros/ros.h>

#include <fstream>

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

#include "common/PathPlanningUtil.h"
#include "common/NavigationUtil.h"
#include "common/LaserScanUtil.h"

#include "common/utilViz.h"
#include "common/utilMap.h"

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
        SM_AVOIDANCE = 8,
        SM_AVOIDANCE_LEFT = 9,
        SM_AVOIDANCE_RIGHT = 10,
        SM_AVOIDANCE_CREATE_POLYGONS = 11,
        SM_FINISH = 12,
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
        laserUtil.initRosConnection(&nh);
        loc_marker_pub = nh.advertise<visualization_msgs::Marker>("sensor_polygon", 10);
        polygons_ptr = 0;
        num_polygons = 0;
    }
    ~MotionPlannerSymAction(){
    }
    void executeCallback(const common::MotionPlannerSymGoalConstPtr msg) {
        bool success = false, preempted = false, correctAngle = false;

        std::cout << "New Goal Motion planner:" << msg->goalPose.x << "," << msg->goalPose.y
                << "," << msg->goalPose.theta << std::endl;

        float currX, currY, currTheta, goalX, goalY, goalTheta;
        biorobotics::Vertex2 lastUpdatePose, currPose;
        bool init = true, replanning = false;
        float ratio = 0.5;

        float timeout = msg->timeout;

        boost::posix_time::ptime prev =
                boost::posix_time::second_clock::local_time();
        boost::posix_time::ptime curr = prev;

        boost::posix_time::ptime time_min;

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

        std::vector<geometry_msgs::Polygon> polygons;
        std::vector<biorobotics::Polygon> polygons_viz;

        ros::Rate rate(10);

        while(ros::ok() && ((curr - prev).total_milliseconds() < timeout || timeout == 0) 
                && state != SM_FINISH){
            if (as->isPreemptRequested()){
                ROS_INFO("%s: Preempted", " Action MotionPlannerSymAction");
                as->setPreempted();
                preempted = true;
                break;
            }
            navigationUtil.getCurrPose(currX, currY, currTheta);
            currPose = biorobotics::Vertex2(currX, currY);
            if(init){
                lastUpdatePose = biorobotics::Vertex2(currX, currY);
                init = false;
            }

            int sensor = 0;
            biorobotics::LaserScan * laserScan = laserUtil.getLaserScan();

            if(laserScan != 0)
                sensor = biorobotics::quantizedInputs(laserScan, currTheta, 0.7, 0.4, M_PI_4);

            switch(state){
                case SM_INIT:
                    indexCurrPath = 0;
                    path = pathPlanningUtil.planPathSymbMapDjsk(currX, currY, goalX, goalY, 
                            polygons, success);
                    if(success){
                        success = false;
                        replanning = false;
                        state = SM_GET_NEXT_POSITION;
                    }
                    else
                        state = SM_FINISH;
                break;
                case SM_GET_NEXT_POSITION:
                    if(indexCurrPath < path.poses.size()){
                        nextX = path.poses[indexCurrPath].pose.position.x;
                        nextY = path.poses[indexCurrPath].pose.position.y;
                        if(indexCurrPath == 0)
                            state = SM_NAV_NEXT_GOAL;
                        else
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
                    time_min = curr;
                    lastUpdatePose = currPose;
                break;
                case SM_WAIT_REACHED_GOAL:
                    if(currPose.sub(lastUpdatePose).norm() > ratio){
                        lastUpdatePose = currPose;
                        time_min = curr;
                    }
                    if((curr - time_min).total_milliseconds() > 6000){
                        navigationUtil.stopMotion();
                        state = SM_AVOIDANCE;
                    }
                    else{
                        if(navigationUtil.finishedCurrMotionPF()){
                            indexCurrPath++;
                            state = SM_GET_NEXT_POSITION;
                        }
                        else
                            state = SM_WAIT_REACHED_GOAL;
                    }
                break;
                case SM_CORRECT_ANGLE:
                    turn = goalTheta - currTheta;
                    if(turn > M_PI) 
                        turn -= 2*M_PI;
                    if(turn <= -M_PI) 
                        turn += 2*M_PI;
                    success = navigationUtil.syncMoveDistAngle(0, turn, 10000);
                    std::cout << "FINSH" << std::endl;
                    state = SM_FINISH;
                    navigationUtil.stopMotion();
                break;
                case SM_AVOIDANCE:
                    if(sensor > 0){
                        if(sensor == 1)
                            state = SM_AVOIDANCE_LEFT;
                        else if(sensor == 2)
                            state = SM_AVOIDANCE_RIGHT;
                        else if(sensor == 3)
                            state = SM_AVOIDANCE_CREATE_POLYGONS;
                        navigationUtil.syncMoveDist(-0.15, false, 0);
                    }
                    else{
                        navigationUtil.syncMoveDist(0.15, false, 0);
                        state = SM_AVOIDANCE_CREATE_POLYGONS;
                    }
                break;
                case SM_AVOIDANCE_LEFT:
                    navigationUtil.syncMoveDist(-0.15, true, 0);
                    state = SM_AVOIDANCE_CREATE_POLYGONS;
                break;
                case SM_AVOIDANCE_RIGHT:
                    navigationUtil.syncMoveDist(0.15, true, 0);
                    state = SM_AVOIDANCE_CREATE_POLYGONS;
                break;
                case SM_AVOIDANCE_CREATE_POLYGONS:
                    polygons = biorobotics::getSensorPolygon(laserScan, currX, currY, currTheta, 1.0, 0.24);
                    polygons_ptr = biorobotics::convertGeometryMsgToPolygons(polygons, 
                            polygons_ptr, &num_polygons);
                    polygons_viz = std::vector<biorobotics::Polygon>(
                            polygons_ptr, polygons_ptr + num_polygons);
                    state = SM_INIT;
                    //boost::this_thread::sleep(boost::posix_time::milliseconds(20000));
                break;
            }
            sendToVizPolygonMarker(polygons_viz, "sensor_polygon", &loc_marker_pub);
            curr = boost::posix_time::second_clock::local_time();
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
    LaserScanUtil laserUtil;

    std::map<std::string, std::vector<float> > locations;

    biorobotics::Polygon * polygons_ptr;
    int num_polygons;

    ros::Publisher loc_marker_pub;
};

class MotionPlannerGridAction {

    enum States{
        SM_INIT = 0,
        SM_GET_NEXT_PATH = 1,
        SM_WAIT_REACHED_PATH = 2,
        SM_CORRECT_ANGLE = 3,
        SM_AVOIDANCE_BACK = 4,
        SM_AVOIDANCE_LEFT = 5,
        SM_AVOIDANCE_RIGHT = 6,
        SM_FINISH = 7
    };

public:
    MotionPlannerGridAction(std::string name, std::map<std::string, std::vector<float> > locations) 
        : action_name(name), locations(locations){
        as = new actionlib::SimpleActionServer<common::MotionPlannerGridAction>(nh, name,
                    boost::bind(&MotionPlannerGridAction::executeCallback, this, _1),
                    false);
        as->start();
        
        laserUtil.initRosConnection(&nh);
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

            biorobotics::LaserScan * laserScan = laserUtil.getLaserScan();
            int sensor = 0;
            if(laserScan != 0)
                sensor = biorobotics::quantizedInputs(laserScan, currTheta, 0.37, 0.29, M_PI_4);
                //sensor = biorobotics::quantizedInputs(laserScan, currTheta, 0.7, 0.4, M_PI_4);
                //sensor = biorobotics::quantizedInputs(laserScan, currTheta, 0.27, 0.3, M_PI_4);

            switch(state){
                case SM_INIT:
                    indexCurrPath = 0;
                    path = pathPlanningUtil.planPathGridMap(currX, currY, goalX, goalY, success);
                    if(success){
                        success = false;
                        state = SM_GET_NEXT_PATH;
                    }
                    else
                        state = SM_FINISH;
                break;
                case SM_GET_NEXT_PATH:
                    //navigationUtil.asyncPotentialFields(nextX, nextY);
                    navigationUtil.asyncMovePath(path);
                    state = SM_WAIT_REACHED_PATH;
                break;
                case SM_WAIT_REACHED_PATH:
                    if(sensor > 0){
                        navigationUtil.stopMotion();
                        state = SM_AVOIDANCE_BACK;
                    }
                    else{
                        if(navigationUtil.finishedCurrMotionPath()){
                            if(correctAngle)
                                state = SM_CORRECT_ANGLE;
                            else{
                                success = true;
                                state = SM_FINISH;
                            }
                        }
                        else
                            state = SM_WAIT_REACHED_PATH;
                    }
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
                case SM_AVOIDANCE_BACK:
                    if(sensor > 0){
                        if(sensor == 1)
                            state = SM_AVOIDANCE_LEFT;
                        else if(sensor == 2)
                            state = SM_AVOIDANCE_RIGHT;
                        else if(sensor == 3)
                            state = SM_INIT;
                        navigationUtil.syncMoveDist(-0.15, false, 0);
                    }
                    else{
                        navigationUtil.syncMoveDist(0.15, false, 0);
                        state = SM_INIT;
                    }
                break;
                case SM_AVOIDANCE_LEFT:
                    navigationUtil.syncMoveDist(-0.15, true, 0);
                    state = SM_INIT;
                break;
                case SM_AVOIDANCE_RIGHT:
                    navigationUtil.syncMoveDist(0.15, true, 0);
                    state = SM_INIT;
                break;
            }
            curr = boost::posix_time::second_clock::local_time();
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
    LaserScanUtil laserUtil;

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

	MotionPlannerSymAction actionSym("motion_planner_action_sym", locations);
    MotionPlannerGridAction actionGtid("motion_planner_action_grid", locations);

    ros::Publisher loc_marker_pub = n.advertise<visualization_msgs::Marker>("loc_marker", 10);

    ros::Rate rate(15);

    while(ros::ok()){

        biorobotics::sendToVizLocationMarker(locations, &loc_marker_pub);

        rate.sleep();
        ros::spinOnce();
    }

    return 1;

}