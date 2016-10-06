
#include "path_planning/VoronoiPlanner.h"

VoronoiPlanner::VoronoiPlanner(){
}
VoronoiPlanner::~VoronoiPlanner(){
}

void VoronoiPlanner::outlineMap(unsigned char* costarr, int nx, int ny, unsigned char value) {
        unsigned char* pc = costarr;
        for (int i = 0; i < nx; i++)
                *pc++ = value;
        pc = costarr + (ny - 1) * nx;
        for (int i = 0; i < nx; i++)
                *pc++ = value;
        pc = costarr;
        for (int i = 0; i < ny; i++, pc += nx)
                *pc = value;
        pc = costarr + nx - 1;
        for (int i = 0; i < ny; i++, pc += nx)
                *pc = value;
}

bool ** VoronoiPlanner::computeMapConvert(DynamicVoronoi *voronoi_,
                costmap_2d::Costmap2D* costmap_) {
        bool **map = NULL;

        int sizeX = costmap_->getSizeInCellsX();
        int sizeY = costmap_->getSizeInCellsY();

        map = new bool*[sizeX];

        ros::Time t = ros::Time::now();

        for (int x = 0; x < sizeX; x++) {
                (map)[x] = new bool[sizeY];
        }

        for (int y = sizeY - 1; y >= 0; y--) {
                for (int x = 0; x < sizeX; x++) {
                        unsigned char c = costmap_->getCost(x, y);

                        if (c == costmap_2d::FREE_SPACE || c == costmap_2d::NO_INFORMATION)
                                (map)[x][y] = false; // cell is free
                        else
                                (map)[x][y] = true; // cell is occupied
                }
        }

        ROS_INFO("Time (for map convert): %f sec", (ros::Time::now() - t).toSec());

        return map;
}

bool VoronoiPlanner::worldToMap(costmap_2d::Costmap2D* costmap_, double wx, double wy,
                double& mx, double& my) {
        double origin_x = costmap_->getOriginX(), origin_y = costmap_->getOriginY();
        double resolution = costmap_->getResolution();

        if (wx < origin_x || wy < origin_y)
                return false;

        float convert_offset_ = 0;
        mx = (wx - origin_x) / resolution - convert_offset_;
        my = (wy - origin_y) / resolution - convert_offset_;

        if (mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY())
                return true;

        return false;
}

void VoronoiPlanner::mapToWorld(costmap_2d::Costmap2D* costmap_, double mx, double my,
                double& wx, double& wy) {
        float convert_offset_ = 0;
        wx = costmap_->getOriginX()
                        + (mx + convert_offset_) * costmap_->getResolution();
        wy = costmap_->getOriginY()
                        + (my + convert_offset_) * costmap_->getResolution();
}

void VoronoiPlanner::clearRobotCell(costmap_2d::Costmap2D* costmap_, unsigned int mx,
                unsigned int my) {
        costmap_->setCost(mx, my, costmap_2d::FREE_SPACE);
}


void VoronoiPlanner::computeVoronoi(DynamicVoronoi *voronoi_, costmap_2d::Costmap2D* costmap_,
                bool ** map) {

	// Pre processing for generalized voronoi diagram
	// Get of dimension of the cost map grid
	int nx = costmap_->getSizeInCellsX(), ny = costmap_->getSizeInCellsY();
	// Generalized voronoi diagram with of bounding box grid dimension
        outlineMap(costmap_->getCharMap(), nx, ny, costmap_2d::LETHAL_OBSTACLE);
	// Before of create generalized vornoi diagram with point of limit create a map
	// Convert costmap to array of bools
        map = computeMapConvert(voronoi_, costmap_);

        bool doPrune = true;

        ros::Time t = ros::Time::now();
	// Get the size of costmap to build voronoi diagram
        int sizeX = costmap_->getSizeInCellsX();
        int sizeY = costmap_->getSizeInCellsY();

        ROS_INFO("voronoi initializeMap");
        voronoi_->initializeMap(sizeX, sizeY, map);
        ROS_INFO("Time for initializeMap: %f sec",
                        (ros::Time::now() - t).toSec());
        t = ros::Time::now();

        ROS_INFO("voronoi update");
        voronoi_->update(); // update distance map and Voronoi diagram
        ROS_INFO("Time for update: %f sec", (ros::Time::now() - t).toSec());
        t = ros::Time::now();

        ROS_INFO("voronoi prune");
        if (doPrune)
                voronoi_->prune();  // prune the Voronoi
        ROS_INFO("Time for prune: %f sec", (ros::Time::now() - t).toSec());
}


bool VoronoiPlanner::computePlan(costmap_2d::Costmap2D* costmap_, DynamicVoronoi * voronoi_,
                const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal, double tolerance,
                std::vector<geometry_msgs::PoseStamped>& plan) {

        //Sure that plan is clear
        plan.clear();

        ros::NodeHandle n;

        double wx = start.pose.position.x;
        double wy = start.pose.position.y;

        unsigned int start_x_i, start_y_i, goal_x_i, goal_y_i;
        double start_x, start_y, goal_x, goal_y;

        if (!costmap_->worldToMap(wx, wy, start_x_i, start_y_i)) {
                ROS_WARN(
                                "The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
                return false;
        }
        worldToMap(costmap_, wx, wy, start_x, start_y);

        wx = goal.pose.position.x;
        wy = goal.pose.position.y;

	if (!costmap_->worldToMap(wx, wy, goal_x_i, goal_y_i)) {
                ROS_WARN_THROTTLE(1.0,
                                "The goal sent to the global planner is off the global costmap. Planning will always fail to this goal.");
                return false;
        }
        worldToMap(costmap_, wx, wy, goal_x, goal_y);

        clearRobotCell(costmap_, start_x_i, start_y_i);
	bool ** map;
        computeVoronoi(voronoi_, costmap_, map);

        ros::Time t_b = ros::Time::now();
        ros::Time t = ros::Time::now();

        std::vector<std::pair<float, float> > path1;
        std::vector<std::pair<float, float> > path2;
        std::vector<std::pair<float, float> > path3;

	ROS_INFO("start_x %f, start_y %f", start_x, start_y);

        bool res1 = false, res2 = false, res3 = false;

	// If goal not are in cell of the vornoi diagram, we have that best findPath of goal to voronoi diagram without have a cell occupancie
	if (!voronoi_->isVoronoi(goal_x, goal_y)) {
                res3 = computeUniformCostPath(&path3, goal_x, goal_y, start_x, start_y, voronoi_, 0,
                                1);
                std::cout << "computePath goal to VD " << res3 << std::endl;
                goal_x = std::get < 0 > (path3[path3.size() - 1]);
                goal_y = std::get < 1 > (path3[path3.size() - 1]);

		ROS_INFO("Is voronoi goal compute %d", voronoi_->isVoronoi(goal_x, goal_y));		

                std::reverse(path3.begin(), path3.end());
        }

        if (!voronoi_->isVoronoi(start_x, start_y)) {
                res1 = computeUniformCostPath(&path1, start_x, start_y, goal_x, goal_y, voronoi_, 0,
                                1);
                std::cout << "computePath start to VD " << res1 << std::endl;
                start_x = std::get < 0 > (path1[path1.size() - 1]);
                start_y = std::get < 1 > (path1[path1.size() - 1]);

		ROS_INFO("Is voronoi start compute %d", voronoi_->isVoronoi(start_x, start_y));		
        }
	
        res2 = computeAStarPath(&path2, start_x, start_y, goal_x, goal_y, voronoi_);
	//res2 = computeUniformCostPath(&path2, start_x, start_y, goal_x, goal_y, voronoi_, 1, 0);
        ROS_INFO("computePath %d", res2);

        if (!(res1 && res2 && res3)) {
                ROS_INFO("Failed to compute full path");
        }

        path1.insert(path1.end(), path2.begin(), path2.end());
        path1.insert(path1.end(), path3.begin(), path3.end());

        /*for (int i = 0; i < path1.size(); i++) {
                int x = std::get < 0 > (path1[i]);
                int y = std::get < 1 > (path1[i]);

                if (x > 0 && y > 0)
                        map[x][y] = 1;
        }*/

	smoothPath(&path1);

        for (int i = 0; i < path1.size(); i++) {

                geometry_msgs::PoseStamped new_goal = goal;
                tf::Quaternion goal_quat = tf::createQuaternionFromYaw(1.54);

                new_goal.pose.position.x = std::get < 0 > (path1[i]);
                new_goal.pose.position.y = std::get < 1 > (path1[i]);

                mapToWorld(costmap_, new_goal.pose.position.x, new_goal.pose.position.y,
                                new_goal.pose.position.x, new_goal.pose.position.y);

                new_goal.pose.orientation.x = goal_quat.x();
                new_goal.pose.orientation.y = goal_quat.y();
                new_goal.pose.orientation.z = goal_quat.z();
                new_goal.pose.orientation.w = goal_quat.w();
                plan.push_back(new_goal);
        }

        ROS_INFO("\nTime to get plan: %f sec\n", (ros::Time::now() - t_b).toSec());

        return !plan.empty();
}

void VoronoiPlanner::smoothPath(std::vector<std::pair<float, float> > *path) {
        // Make a deep copy of path into newpath
        std::vector<std::pair<float, float> > newpath = *path;

        float tolerance = 0.00001;
        float change = tolerance;

        if (path->size() < 2)
                return;

        while (change >= tolerance) {

                change = 0.0;
                for (int i = 1; i < path->size() - 1; i++) {
                        float aux_x = newpath[i].first;
                        float aux_y = newpath[i].second;

                        float newpath_x = newpath[i].first
                                        + 0.5 * (path->at(i).first - newpath[i].first);
                        float newpath_y = newpath[i].second
                                        + 0.5 * (path->at(i).second - newpath[i].second);

                        newpath_x = newpath_x
                                        + 0.3
                                                        * (newpath[i - 1].first + newpath[i + 1].first
                                                                        - (2.0 * newpath_x));
                        newpath_y = newpath_y
                                        + 0.3
                                                        * (newpath[i - 1].second + newpath[i + 1].second
                                                                        - (2.0 * newpath_y));

                        change = change + fabs(aux_x - newpath_x);
                        change = change + fabs(aux_y - newpath_y);

                        newpath[i] = std::make_pair(newpath_x, newpath_y);
                }
        }
        *path = newpath;
}

bool VoronoiPlanner::computeUniformCostPath(std::vector<std::pair<float, float> > *path, int init_x,
		int init_y, int goal_x, int goal_y, DynamicVoronoi *voronoi,
		bool check_is_voronoi_cell, bool stop_at_voronoi) {

	unsigned int sizeX = voronoi->getSizeX();
	unsigned int sizeY = voronoi->getSizeY();

	std::vector<std::pair<int, int> > delta;
	delta.push_back( { -1, 0 });     	// go up
	delta.push_back( { 0, -1 });     	// go left
	delta.push_back( { 1, 0 });      	// go down
	delta.push_back( { 0, 1 });      	// go right
	delta.push_back( { -1, -1 });    	// up and left
	delta.push_back( { -1, 1 });    	// up and right
	delta.push_back( { 1, -1 });     	// down and left
	delta.push_back( { 1, 1 });			// down and right

	// closed cells grid (same size as map grid)
	bool **closed = NULL;
	closed = new bool*[sizeX];
	for (int x = 0; x < sizeX; x++) {
		(closed)[x] = new bool[sizeY];
	}
	// Closed cells grid init false.
	for (int y = sizeY - 1; y >= 0; y--) {
		for (int x = 0; x < sizeX; x++) {
			(closed)[x][y] = false;
		}
	}

	// actions (number of delta's row) cells grid (same size as map grid)
	int **action = NULL;
	action = new int*[sizeX];
	for (int x = 0; x < sizeX; x++) {
		(action)[x] = new int[sizeY];
	}
	for (int y = sizeY - 1; y >= 0; y--) {
		for (int x = 0; x < sizeX; x++) {
			(action)[x][y] = -1;
		}
	}

	// The current cell is the init
	int x = init_x;
	int y = init_y;

	// Set cost to 0
	float g = 0;
	// Set the cost between of Neighbour cells is 1
	float cost = 1;

	// Vector of open (for possible expansion) nodes
	std::vector < std::tuple<float, int, int> > open;
	open.push_back(std::make_tuple(g, x, y));

	// Path found flag
	bool found = false;
	// No solution could be found flag
	bool resign = false;

	while (!found && !resign) {
		if (open.size() == 0) {
			resign = true;
			path->empty();
			return false;
		} else {
			// Sort open by cost
			sort(open.begin(), open.end());
			reverse(open.begin(), open.end());
			// Get the node with lowest cost
			std::tuple<float, int, int> next = open[open.size() - 1];
			open.pop_back();
			g = std::get < 0 > (next);
			x = std::get < 1 > (next);
			y = std::get < 2 > (next);

			// check, whether the solution is found (we are at the goal)
			if (stop_at_voronoi) {
				// if stop_at_voronoi is set, we stop, when get path to any voronoi cell
				if (voronoi->isVoronoi(x, y)) {
					found = true;
					goal_x = x;
					goal_y = y;
					continue;
				}
			} else {
				if (x == goal_x && y == goal_y) {
					found = true;
					continue;
				}
			}
			// Iterator for node espansion, the Neighbour of the current cell
			for (int i = 0; i < delta.size(); i++) {
				// Expansion
				int x2 = x + std::get < 0 > (delta[i]);
				int y2 = y + std::get < 1 > (delta[i]);

				//Check that new node expansion to be in grid bounds
				if (x2 >= 0 && x2 < sizeX && y2 >= 0 && y2 < sizeY) {
					// Check new node expasion not to be in obstacle
					if (voronoi->isOccupied(x2, y2)) {
						continue;
					}
					// Check new node expasion was not early visited
					if (closed[x2][y2]) {
						continue;
					}

					// check new node is on Voronoi diagram
					if (!voronoi->isVoronoi(x2, y2) && check_is_voronoi_cell) {
						continue;
					}

					float g2 = g + cost;
					open.push_back(std::make_tuple(g2, x2, y2));
					closed[x2][y2] = true;
					action[x2][y2] = i;
				}
			}
		}
	}

	// Make reverse steps from goal to init to write path
	x = goal_x;
	y = goal_y;

	int i = 0;
	path->clear();

	while (x != init_x || y != init_y) {
		path->push_back( { x, y });
		i++;

		int x2 = x - std::get < 0 > (delta[action[x][y]]);
		int y2 = y - std::get < 1 > (delta[action[x][y]]);

		x = x2;
		y = y2;
	}

	reverse(path->begin(), path->end());
	return true;
}

bool VoronoiPlanner::computeAStarPath(std::vector<std::pair<float, float> > *path, int init_x,
                int init_y, int goal_x, int goal_y, DynamicVoronoi *voronoi){
        unsigned int sizeX = voronoi->getSizeX();
        unsigned int sizeY = voronoi->getSizeY();

        std::vector<std::pair<int, int> > delta;
        delta.push_back( { -1, 0 });            // go up
        delta.push_back( { 0, -1 });            // go left
        delta.push_back( { 1, 0 });             // go down
        delta.push_back( { 0, 1 });             // go right
        delta.push_back( { -1, -1 });           // up and left
        delta.push_back( { -1, 1 });            // up and right
        delta.push_back( { 1, -1 });            // down and left
        delta.push_back( { 1, 1 });             // down and right

        float g, f, h;
        g = 0;
        h = fabs(init_x - goal_x) + fabs(init_y - goal_y);
        f = g + h;

        NodeAStar * start = new NodeAStar(init_x, init_y, f, g, h, nullptr);

        std::vector < std::tuple<float, NodeAStar *> > open;

        open.push_back(std::make_tuple(f, start));

        std::vector <NodeAStar *> closed;
        NodeAStar * goalNode = nullptr;

        bool foundPath = false;
        while(open.size() > 0 && !foundPath){
                // Sort open by cost
                sort(open.begin(), open.end());
                reverse(open.begin(), open.end());
                std::tuple<float, NodeAStar *> tuple = open[open.size() - 1];
                open.pop_back();

                NodeAStar * current = std::get <1> (tuple);

                //std::cout << "Current node:" << current->x << "," << current->y << std::endl;

                if (current->x == goal_x && current->y == goal_y) {
                        goalNode = current;
                        foundPath = true;
                        continue;
                }

                closed.push_back(current);

                for (int i = 0; i < delta.size(); i++) {

                        int xn = current->x + std::get < 0 > (delta[i]);
                        int yn = current->y + std::get < 1 > (delta[i]);

                        //std::cout << "Neighbor node:" << xn << "," << yn << std::endl;

                        if (xn >= 0 && xn < sizeX && yn >= 0 && yn < sizeY) {
                                // Check new node expasion not to be in obstacle
                                if (voronoi->isOccupied(xn, yn)) {
                                        continue;
                                }

                                // check new node is on Voronoi diagram
                                if (!voronoi->isVoronoi(xn, yn)) {
                                        continue;
                                }
                        }

                        //std::cout << "Is a voronoi cell" << std::endl;

                        NodeAStar * neighbor = new NodeAStar();

                        neighbor->x = xn;
                        neighbor->y = yn;

                        neighbor->h = fabs(neighbor->x - goal_x) + fabs(neighbor->y - goal_y);
                        if(fabs(std::get < 0 > (delta[i])) == 1 && fabs(std::get < 1 > (delta[i])) == 1){
                                //std::cout << "Diagonal:" << std::endl;
                                neighbor->g = current->g +  1.414;
                        }
                        else{
                                //std::cout << "Lateral:" << std::endl;
                                neighbor->g = current->g +  1.0;
                        }
                        neighbor->parent = current;

                        bool found = false;
                        for(int j = 0; j < closed.size() && !found; j++){
                                if(neighbor->x == closed[j]->x && neighbor->y == closed[j]->y)
                                        found = true;
                        }

                        if(!found){
                                //std::cout << "Not Found in the closed list:" << std::endl; 
                                neighbor->f = neighbor->g + neighbor->h;
                                int indexFound = -1;
                                for(int j = 0; j < open.size() && indexFound < 0; j++){
                                        int xon = std::get<1>(open[j])->x;
                                        int yon = std::get<1>(open[j])->y;
                                        if(xon == neighbor->x && yon == neighbor->y)
                                                indexFound = j;
                                }
                                if(indexFound < 0){
                                        //std::cout << "Insert node in open list" << std::endl;
                                        open.push_back(std::make_tuple(neighbor->f, neighbor));
                                }
                                else{
                                        //std::cout << "Have found a node in open list" << std::endl;
                                        if(neighbor->g < std::get<1>(open[indexFound])->g){
                                                std::get<1>(open[indexFound])->g = neighbor->g;
                                                std::get<1>(open[indexFound])->parent = neighbor->parent;
                                        }
                                }
                        }
                }
        }
        if(foundPath){
                std::cout << "Found path:" << std::endl;
                NodeAStar * node = goalNode;
                while(node != nullptr){
                        path->push_back( { node->x, node->y });
                        node = node->parent;
                }
                reverse(path->begin(), path->end());
                return true;
        }
        return false;
}

nav_msgs::Path VoronoiPlanner::makePathFromPoses(const std::vector<geometry_msgs::PoseStamped>& path_poses){
	nav_msgs::Path path;
        path.poses.resize(path_poses.size());

        if (!path_poses.empty()) {
                path.header.frame_id = "/map";
                path.header.stamp = ros::Time::now();
        }

        // Extract the plan in world co-ordinates, we assume the path is all in the same frame
        for (unsigned int i = 0; i < path_poses.size(); i++) {
                path.poses[i] = path_poses[i];
        }

        return path;
}

void VoronoiPlanner::publishVoronoiGrid(DynamicVoronoi *voronoi_,
		costmap_2d::Costmap2D* costmap_, std::string frame_id_, ros::Publisher * voronoi_grid_pub_) {
	int nx = costmap_->getSizeInCellsX(), ny = costmap_->getSizeInCellsY();

	ROS_WARN("costmap sx = %d,sy = %d, voronoi sx = %d, sy = %d", nx, ny,
			voronoi_->getSizeX(), voronoi_->getSizeY());

	double resolution = costmap_->getResolution();
	nav_msgs::OccupancyGrid grid;
	// Publish Whole Grid
	grid.header.frame_id = frame_id_;
	grid.header.stamp = ros::Time::now();
	grid.info.resolution = resolution;

	grid.info.width = nx;
	grid.info.height = ny;

	double wx, wy;
	costmap_->mapToWorld(0, 0, wx, wy);
	grid.info.origin.position.x = wx - resolution / 2;
	grid.info.origin.position.y = wy - resolution / 2;
	grid.info.origin.position.z = 0.0;
	grid.info.origin.orientation.w = 1.0;

	grid.data.resize(nx * ny);

	for (unsigned int x = 0; x < nx; x++) {
		for (unsigned int y = 0; y < ny; y++) {
			if (voronoi_->isVoronoi(x, y))
				grid.data[x + y * nx] = 128;
			else
				grid.data[x + y * nx] = 0;
		}
	}
	voronoi_grid_pub_->publish(grid);
}
