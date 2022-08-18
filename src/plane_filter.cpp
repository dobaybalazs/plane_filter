#include "plane_filter/plane_filter.h"


PlaneFilter::PlaneFilter(ros::NodeHandlePtr nh){
    sub = nh->subscribe(params::input_topic,1,&PlaneFilter::callBack,this);

    pub = nh->advertise<pcl::PCLPointCloud2>("/output",1);
}

void PlaneFilter::voxelFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    pcl::VoxelGrid<pcl::PointXYZ> grid;
    grid.setInputCloud(cloud);
    grid.setLeafSize(params::lx,params::ly,params::lz);
    grid.setDownsampleAllData(true);
    grid.filter(*cloud);
}

void PlaneFilter::boxFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices ());
    pcl::CropBox<pcl::PointXYZ> roi(true);

    Eigen::Vector4f min(params::min_x,params::min_y,params::min_z,0.0);
    Eigen::Vector4f max(params::max_x,params::max_y,params::max_z,0.0);

    roi.setMin(min);
    roi.setMax(max);
    roi.setInputCloud(cloud);
    roi.filter(inliers->indices);

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setIndices(inliers);
    extract.setInputCloud(cloud);
    extract.filter(*cloud);
}



void PlaneFilter::filterPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    pcl::PointCloud<pcl::PointXYZ>::Ptr inlier_points        (new pcl::PointCloud<pcl::PointXYZ>);

    // Object for Line fitting
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr      inliers      (new pcl::PointIndices ());

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients (true);       
    seg.setInputCloud (cloud);                
    seg.setModelType (pcl::SACMODEL_PLANE);    
    seg.setMethodType (pcl::SAC_RANSAC);      
    seg.setMaxIterations (1000);              
    seg.setDistanceThreshold (params::ransac_dist);       
    seg.setRadiusLimits(params::rradius_min, params::rradius_max);            
    seg.segment (*inliers, *coefficients);    

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setIndices (inliers);
    extract.setNegative (params::invertFilter);
    extract.filterDirectly (cloud);
}

void PlaneFilter::callBack(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    if(params::useBoxFilter)
        PlaneFilter::boxFilter(cloud);

    if(params::useVoxelF)
        PlaneFilter::voxelFilter(cloud);
    
    if(params::usePlaneFilter)
        PlaneFilter::filterPlane(cloud);

    pub.publish(cloud);
}
