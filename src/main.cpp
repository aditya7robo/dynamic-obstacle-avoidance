#include <ros/ros.h>
#include <imminent_collision/imminent_collision.hpp>


int main(int argc,char** argv)
{
    ros::init(argc, argv, "ica_node");
    ros::NodeHandle nh;
    imminentCollisionAction ica(nh);
    ROS_INFO("[ICA MAIN] initialized ICA");
    ros::Rate rate(10);
    while(ros::ok())
    {    
        ros::spinOnce();
        ica.tick();
        rate.sleep();
    }
}