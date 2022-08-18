#pragma once

//ROS
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include "plane_filter/plane_filterConfig.h"
#include <visualization_msgs/Marker.h>

//PCL
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/voxel_grid.h>

//STL
#include <string>

namespace params{
    extern std::string input_topic;
    extern float lx,ly,lz;
    extern float min_x,min_y,min_z;
    extern float max_x,max_y,max_z;
    extern bool usePlaneFilter,invertFilter,useVoxelF, useBoxFilter;
    extern float ransac_dist, rradius_min,rradius_max;
};

class PlaneFilter{
    ros::Subscriber sub;
    ros::Publisher pub;

    public:
    PlaneFilter(ros::NodeHandlePtr);

    void voxelFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr);

    void boxFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr);

    void filterPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr);

    void callBack(pcl::PointCloud<pcl::PointXYZ>::Ptr);
};