#ifndef PATH_PLANNING_H
#define PATH_PLANNING_H

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <string>

#include "tf/tf.h"
#include "tf/transform_listener.h"
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

using namespace cv;
using namespace std;

constexpr double PI = 3.14159;

struct CellIndex {
    int row;
    int col;
    double theta; // {0, 45, 90, 135, 180, 225, 270, 315}
};

class PathPlanning {
public:
    PathPlanning(costmap_2d::Costmap2DROS *costmap2dRos);

    vector<geometry_msgs::PoseStamped> getPathInROS(); // Get the final path in ROS

    void publishCoveragePath(); // For visualization

    int getSizeOfCell() { return this->m_cellSize; } // Get the size of a cell, must be an odd number

private:
    void initMat(); // Initialize m_cellMat and m_neuralMat
    void initCoveredGrid(); // Initialize m_coveredPathGrid
    void getPathInCV(); // Use OpenCV API to calculate the planned path, then convert it to ROS
    bool boundingJudge(int a, int b); // Bounding box judge
    void publishPlan(const std::vector<geometry_msgs::PoseStamped> &path);

    bool m_initialized; // Flag indicating if the class is successfully initialized
    costmap_2d::Costmap2DROS *m_cosmap2dRos; // A ROS wrapper for a 2D Costmap, handles topics that provide obstacle observations in the form of PointCloud or LaserScan messages.
    costmap_2d::Costmap2D* m_costmap2d; // Pointer to the raw costmap
    Mat m_srcMap; // Raw costmap converted to Mat
    Mat m_cellMat; // Grid map merged by m_CELL_SIZE
    Mat m_neuralMat; // This map is used for path planning
    vector<CellIndex> m_freeSpaceVec; // Free space in m_cellMat
    nav_msgs::OccupancyGrid m_coveredPathGrid; // Occupancy grid map, 0 indicates unoccupied, 1 indicates occupied, -1 indicates unknown

    geometry_msgs::PoseStamped m_initPose; // Initial pose
    vector<CellIndex> m_pathVec; // Path obtained using OpenCV API
    vector<geometry_msgs::PoseStamped> m_pathVecInROS; // m_pathVec converted to ROS path

    ros::Publisher m_planPub; // Publisher for planned path, displayed in rviz
    ros::Publisher m_gridPub; // Publisher for robot's covered path, displayed in rviz

    int m_cellSize; // New grid size, must be an odd multiple of the original grid size
    int m_gridCoveredValue;
};

#endif // PATH_PLANNING_H
