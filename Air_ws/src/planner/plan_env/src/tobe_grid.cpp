#include "plan_env/grid_map.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gridmap");
    ros::NodeHandle nh("~");

    GridMap::Ptr grid_map_;
    grid_map_.reset(new GridMap);
    grid_map_->initMap(nh);

    while(ros::ok())
    {
        timeval start;gettimeofday(&start,NULL);
        ros::spinOnce();
        // double ms;
        // do
        // {
        //     timeval end;gettimeofday(&end,NULL);
        //     ms=1000*(end.tv_sec-start.tv_sec)+0.001*(end.tv_usec-start.tv_usec);
        // }while(ms < 50);
    }
    return 0;
}