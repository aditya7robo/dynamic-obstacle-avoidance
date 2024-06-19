#pragma once

#include <string>
#include <memory>
#include <map>
#include <queue>
#include <vector>
#include <functional>

// #include "behaviortree_cpp/decorator_node.h"
#include <behaviortree_cpp/condition_node.h>
// #include <seven_bt_plugins/protected_queue.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/buffer.h>
#include <seven_costmap_utils/costmap_subscriber.hpp>
#include <seven_costmap_utils/footprint_subscriber.hpp>
#include <seven_costmap_utils/footprint_collision_checker.hpp>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <costmap_2d/costmap_2d.h>
#include <seven_robotics_msgs/Distance.h>
#include <seven_robotics_msgs/ObstacleVelocity.h>
#include <seven_robotics_msgs/ObstacleRadius.h>
#include <seven_robotics_msgs/ObstaclePose.h>
#include <seven_robotics_msgs/EgoVelocity.h>
#include <seven_robot_utils/delegate.hpp>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <obstacle_detector/Obstacles.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <costmap_converter/ObstacleArrayMsg.h>

#include <mbf_msgs/ExePathAction.h>
// #include <seven_bt_plugins/action_node.hpp>

#include <Eigen/Geometry>

//FCL library imports
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <fcl/broadphase/broadphase_continuous_collision_manager.h>
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/distance.h>
#include <fcl/narrowphase/collision_object.h>
#include <fcl/narrowphase/collision_result.h>
#include <fcl/narrowphase/continuous_collision.h>
#include <fcl/narrowphase/continuous_collision_object.h>
#include <fcl/narrowphase/continuous_collision_request.h>
#include <fcl/narrowphase/continuous_collision_result.h>
#include <fcl/geometry/shape/box.h>
#include <fcl/geometry/shape/sphere.h>

// #define ROBOT_RADIUS 0.6

struct dynamicObstacle
{
    geometry_msgs::Point pos_;
    geometry_msgs::PoseStamped poseStamp_;
    geometry_msgs::Twist velocity_;

    int id_;
    float radius_;
    double cp_dist_; //Closest point distance.
    double cp_dt_;
    int fade_counter_;

    void operator=(const dynamicObstacle& rhs)
    {
        id_ = rhs.id_;
        pos_ = rhs.pos_;
        poseStamp_ = rhs.poseStamp_;
        velocity_ = rhs.velocity_;
        radius_ = rhs.radius_;
        cp_dist_ = rhs.cp_dist_;
        cp_dt_ = rhs.cp_dt_;
        fade_counter_ = rhs.fade_counter_;
    }
};


struct CompareByDistance {
    bool operator()(const std::pair<double, int>& lhs, const std::pair<double, int>& rhs) const {
        return lhs.first < rhs.first;
    }
};

typedef std::map<int, dynamicObstacle>::iterator dynObsIter;

class imminentCollisionAction //: public BT::ConditionNode
{

    public:

    imminentCollisionAction(ros::NodeHandle& nh);
    
    void tick();
    float ego_radius = 0.6;

    double calculateRelativeVel(seven_robotics_msgs::ObstacleVelocity& obstacle_velocity,seven_robotics_msgs::EgoVelocity& ego_velocity);
    // void syncCallback(seven_robotics_msgs::Distance::ConstPtr obstacle_distance, seven_robotics_msgs::ObstacleVelocity::ConstPtr obstacle_velocity, seven_robotics_msgs::EgoVelocity::ConstPtr ego_velocity,seven_robotics_msgs::ObstacleRadius::ConstPtr obstacle_radius, seven_robotics_msgs::ObstaclePose::ConstPtr obstacle_pose);
    void projectPoses(seven_robotics_msgs::ObstaclePose& obstacle_pose, seven_robotics_msgs::ObstacleVelocity& obstacle_velocity);

    //CP funtions
    void odomCB(const nav_msgs::Odometry& odom);
    void obstacleCB(const costmap_converter::ObstacleArrayMsg& msg);
    void fadeObstacles(); //To remove obstacles after few iterations of invisibility 
    void removeStaleObstacles(); //Removes faded out obstacles.

    double cpDistanceIntersection(geometry_msgs::Pose& ego_pose_, geometry_msgs::Twist& ego_velocity_, geometry_msgs::Pose& obs_pose_, geometry_msgs::Twist& obs_velocity_); //To calculate the distance at which they will collide.

    void egoPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& ego_pose_);

    int num_steps = 20; //Number of Time Steps.
    float time_interval = 0.5; //Time interval over which to check for collisions.
 
//  private:
 
    // std::shared_ptr<tf2_ros::Buffer> tf;
    // std::shared_ptr<ros::NodeHandle> nh_;

    std::map<int, dynamicObstacle> dynamic_obstacle_;
    double cp_distance;
    // dynamicElement dynObsDistance;
    std::multiset<std::pair<double, int>, CompareByDistance> distance_set;
    // std::priority_queue<dynamicElement, std::vector<dynamicElement>, Comparator> dynObsQueue;
    // iterable_priority_queue<dynamicElement> dynObsQueue;

    void insertObstacle(double distance, const dynamicObstacle& obs);
    void removeObstacleById(int id);

    seven_robotics_msgs::Distance obs_distance;
    seven_robotics_msgs::ObstacleVelocity obs_velocity;
    seven_robotics_msgs::EgoVelocity ego_velocity;
    seven_robotics_msgs::ObstacleRadius obs_radius;
    seven_robotics_msgs::ObstaclePose obs_pose;
    geometry_msgs::PoseWithCovarianceStamped ego_pose;

    geometry_msgs::Twist current_robot_vel_;

    geometry_msgs::PoseArray obs_pose_array;
    geometry_msgs::PoseStamped obs_projected_pose;
    geometry_msgs::PoseStamped ego_projected_pose;

    ros::Subscriber odom_sub_;
    ros::Subscriber ego_pose_sub_;
    ros::Publisher ego_vel_pub_;
    ros::Subscriber obstacle_sub_;

    ros::Publisher obstacle_poseArray_pub_;
    ros::Publisher obstacle_projection_pub_;
    ros::Publisher ego_projection_pub_;

    ros::Publisher refined_path_pub_;
    ros::Publisher collision_path_pub_;

    ros::Publisher should_pause_;

    // message_filters::Subscriber<seven_robotics_msgs::Distance> obstacle_distance_sub;
    // message_filters::Subscriber<seven_robotics_msgs::ObstacleVelocity> obstacle_rel_vel_sub;
    // message_filters::Subscriber<seven_robotics_msgs::EgoVelocity> ego_vel_sub;
    // message_filters::Subscriber<seven_robotics_msgs::ObstacleRadius> obstacle_rad_sub;
    // message_filters::Subscriber<seven_robotics_msgs::ObstaclePose> obstacle_pose_sub;

    // typedef message_filters::sync_policies::ApproximateTime<seven_robotics_msgs::Distance, seven_robotics_msgs::ObstacleVelocity, seven_robotics_msgs::EgoVelocity, seven_robotics_msgs::ObstacleRadius, seven_robotics_msgs::ObstaclePose> MySyncPolicy;
    // typedef message_filters::Synchronizer<MySyncPolicy> SynchronizerType;
    // boost::shared_ptr<SynchronizerType> sync;

    mbf_msgs::ExePathGoal goal_;
    nav_msgs::Path path_;
    nav_msgs::Path collision_path_;

    // ros::Duration limit(30.0);

    std_msgs::Bool is_cancel_control_;

    std_msgs::Bool collision_detected_;

    // typedef message_filters::sync_policies::ExactTime<seven_robotics_msgs::Distance, seven_robotics_msgs::ObstacleVelocity, seven_robotics_msgs::EgoVelocity, seven_robotics_msgs::ObstacleRadius> MySyncPolicy;
    // typedef message_filters::Synchronizer<MySyncPolicy> SynchronizerType;
    // boost::shared_ptr<SynchronizerType> sync;
    // std::shared_ptr<message_filters::TimeSynchronizer<seven_robotics_msgs::Distance, seven_robotics_msgs::ObstacleVelocity>> sync;


    // fcl::Sphere<double> robot_bound (ego_radius);
    // std::shared_ptr<fcl::Sphere<double>> robot_sphere_ptr = std::make_shared<fcl::Sphere<double>>(robot_bound);
    // fcl::CollisionObject<double> robot_object(robot_sphere_ptr);

    // std::shared_ptr<ProtectedQueue<geometry_msgs::PoseStamped>> waypoints_;
    // nav_msgs::Path curr_path, old_path;

        
    bool first_ = false;
    // bool initialized = false;

    private:
    ros::NodeHandle& nh_;
};
