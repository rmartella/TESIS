
#include <ros/ros.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <nav_msgs/Path.h>
#include <dynamicvoronoi.h>
#include <tuple>
#include <vector>
#include <iostream>
#include <string>
#include <tuple>

#include <tf/transform_datatypes.h>

typedef struct _NodeAStar{
	int x, y;
	float f, g, h;
	_NodeAStar * parent;
	_NodeAStar(int x, int y, float f, float g, float h, _NodeAStar * parent){
		this->x = x;
		this->y = y;
		this->f = f;
		this->g = g;
		this->h = h;
		this->parent = parent;
	}
	_NodeAStar(){
	}
}NodeAStar;

class VoronoiPlanner{
public:
       	VoronoiPlanner();
       	~VoronoiPlanner();

	bool computePlan(costmap_2d::Costmap2D* costmap_, DynamicVoronoi * voronoi_,
                const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal, double tolerance,
                std::vector<geometry_msgs::PoseStamped>& plan);
	void publishVoronoiGrid(DynamicVoronoi *voronoi_,
		costmap_2d::Costmap2D* costmap_, std::string frame_id_,
		ros::Publisher * voronoi_grid_pub_);
	nav_msgs::Path makePathFromPoses(const std::vector<geometry_msgs::PoseStamped>& path_poses);

private:
	void outlineMap(unsigned char* costarr, int nx, int ny, unsigned char value);
	bool ** computeMapConvert(DynamicVoronoi *voronoi_,
                costmap_2d::Costmap2D* costmap_);
	void clearRobotCell(costmap_2d::Costmap2D* costmap_, unsigned int mx,
                unsigned int my);
	bool worldToMap(costmap_2d::Costmap2D* costmap_, double wx, double wy,
                double& mx, double& my);
        void mapToWorld(costmap_2d::Costmap2D* costmap_, double mx, double my,
                double& wx, double& wy);
        void computeVoronoi(DynamicVoronoi * voronoi_, costmap_2d::Costmap2D* costmap_,
                bool ** map);
	bool computeUniformCostPath(std::vector<std::pair<float, float> > *path, int init_x,
		int init_y, int goal_x, int goal_y, DynamicVoronoi *voronoi,
		bool check_is_voronoi_cell, bool stop_at_voronoi);
	bool computeAStarPath(std::vector<std::pair<float, float> > *path, int init_x,
		int init_y, int goal_x, int goal_y, DynamicVoronoi *voronoi);
	void smoothPath(std::vector<std::pair<float, float> > *path);

};

