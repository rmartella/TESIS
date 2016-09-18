#include <ros/ros.h>

#include <fstream>

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

#include "common/PathPlanningUtil.h"
#include "common/NavigationUtil.h"

#include "common/MotionPlannerXYA.h"
#include "common/MotionPlannerLoc.h"

std::map<std::string, std::vector<float> > locations;

PathPlanningUtil pathPlanningUtil;
NavigationUtil navigationUtil;

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

bool callbackMotionPlannerXYA(common::MotionPlannerXYA::Request &req, common::MotionPlannerXYA::Request &resp){
	bool success;
	nav_msgs::Path path = pathPlanningUtil.planPathSymbMapDjsk(0.0, 0.0, req.goalPose.x, req.goalPose.y, success);
	navigationUtil.asyncMovePath(path);
}


int main(int argc, char ** argv){

	ros::init(argc, argv, "motion_planner_node");

	ros::NodeHandle n;

	std::string file_location;
	n.getParam("file_location", file_location);


	pathPlanningUtil.initRosConnection(&n);
	navigationUtil.initRosConnection(&n);

	locations = loadKnownLocations(file_location);

	boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

	bool success;
	nav_msgs::Path path;
	path = pathPlanningUtil.planPathSymbMapDjsk(0, 0, -3.95, -4, success);
	//path = pathPlanningUtil.planPathGridMapDjsk(0.0, 0.0, -2.5, 9.57, success);
	if(success){
		//navigationUtil.asyncMovePath(path);
		navigationUtil.syncMovePath(path, 5000);
	}

}