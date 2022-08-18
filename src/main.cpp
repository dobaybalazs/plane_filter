#include "plane_filter/plane_filter.h"

std::string params::input_topic = "";
float params::min_x = -4.0;
float params::min_y = -12.0;
float params::min_z = -1.8;
float params::max_x = 100.0;
float params::max_y = 12.0;
float params::max_z = -0.8;
float params::lx = 0.2;
float params::ly = 0.2;
float params::lz = 0.2;
bool params::useVoxelF = false;
bool params::usePlaneFilter= false;
bool params::useBoxFilter= true;
bool params::invertFilter= true;
float params::ransac_dist = 0.2;
float params::rradius_min = 0.0;
float params::rradius_max = 1.0;

void setParams(plane_filter::plane_filterConfig &config,uint32_t level){
    params::min_x = config.min_x;
    params::min_y = config.min_y;
    params::min_z = config.min_z;
    params::max_x = config.max_x;
    params::max_y = config.max_y;
    params::max_z = config.max_z;
    params::usePlaneFilter= config.usePlaneFilter;
    params::invertFilter = config.invertFilter;
    params::ransac_dist = config.ransac_dist;
    params::rradius_min = config.rradius_min;
    params::rradius_max = config.rradius_max;
    params::useVoxelF = config.useVoxelF;
    params::lx = config.lx;
    params::ly = config.ly;
    params::lz = config.lz;
    params::useBoxFilter = config.useBoxFilter;
}


int main(int argc,char** argv){
    ros::init(argc,argv,"plane_filter_node",ros::init_options::AnonymousName);
    ros::NodeHandlePtr nh = boost::make_shared<ros::NodeHandle>();

    nh->getParam("input_cloud",params::input_topic);

    dynamic_reconfigure::Server<plane_filter::plane_filterConfig> server;
    dynamic_reconfigure::Server<plane_filter::plane_filterConfig>::CallbackType f;

    f = boost::bind(&setParams,_1,_2);
    server.setCallback(f); 

    PlaneFilter pf(nh);

    ros::spin();
}