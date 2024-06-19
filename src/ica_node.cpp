#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <seven_robot_utils/tf_util.hpp>
#include <seven_robot_utils/geometry_utils.hpp>
#include <costmap_2d/cost_values.h>
#include <cmath>
#include <numeric>
#include <message_filters/time_synchronizer.h>
#include <vector>

#include <imminent_collision/imminent_collision.hpp>

imminentCollisionAction::imminentCollisionAction(ros::NodeHandle& nh) : nh_(nh)
{   
    odom_sub_ = nh_.subscribe("/odometry/filtered", 1,&imminentCollisionAction::odomCB,this,ros::TransportHints().tcpNoDelay());
    obstacle_sub_ = nh_.subscribe("/move_base_flex/TebLocalPlannerROS/obstacles",1,&imminentCollisionAction::obstacleCB,this,ros::TransportHints().tcpNoDelay());
    ego_pose_sub_ = nh_.subscribe("/amcl_pose", 1, &imminentCollisionAction::egoPoseCallback, this, ros::TransportHints().tcpNoDelay());

    // obstacle_distance_sub.subscribe(*nh_,"obs_dist",1);
    // obstacle_rel_vel_sub.subscribe(*nh_,"obs_vel_rel",1);
    // ego_vel_sub.subscribe(*nh_,"robot_velocity",1);
    // obstacle_rad_sub.subscribe(*nh_,"obstacles/radius",1);
    // obstacle_pose_sub.subscribe(*nh_, "obs_pose", 1);
    // sync.reset(new SynchronizerType(MySyncPolicy(10), obstacle_distance_sub, obstacle_rel_vel_sub, ego_vel_sub, obstacle_rad_sub, obstacle_pose_sub));
    // sync.registerCallback(boost::bind(&imminentCollisionAction::syncCallback, this, _1, _2, _3, _4, _5));
    
    obstacle_poseArray_pub_ = nh_.advertise<geometry_msgs::PoseArray>("obs_pose_array", 1);
    obstacle_projection_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("obs_projection", 1);
    ego_projection_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("robot_projection", 1);
    
    ego_vel_pub_ = nh_.advertise<seven_robotics_msgs::EgoVelocity>("robot_velocity", 1);

    refined_path_pub_ = nh_.advertise<nav_msgs::Path>("refined_path", 1);
    collision_path_pub_ = nh_.advertise<nav_msgs::Path>("collision_path", 1);

    should_pause_ = nh_.advertise<std_msgs::Bool>("should_pause", 1);
    is_cancel_control_.data = false;
}

double imminentCollisionAction::calculateRelativeVel(seven_robotics_msgs::ObstacleVelocity& obstacle_velocity, seven_robotics_msgs::EgoVelocity& ego_velocity)
{
    double x = obstacle_velocity.relative_velocity.linear.x - ego_velocity.velocity.linear.x;
    double y = obstacle_velocity.relative_velocity.linear.y - ego_velocity.velocity.linear.y;
    double z = obstacle_velocity.relative_velocity.linear.z - ego_velocity.velocity.linear.z;
    double resultant_vel = sqrt((x*x) + (y*y) + (z*z));
    ROS_INFO("!!!!!!!!!!!!!!! [IMMINENT COLLISION] Relative Vel: %f !!!!!!!!!!!!!!!", resultant_vel);
    ROS_INFO("!!!!!!!!!!!!!!! [IMMINENT COLLISION] v_x = %f !!!!!!!!!!!!!!!", x);
    ROS_INFO("!!!!!!!!!!!!!!! [IMMINENT COLLISION] v_y = %f !!!!!!!!!!!!!!!", y);
    
    return resultant_vel;
}

void imminentCollisionAction::projectPoses(seven_robotics_msgs::ObstaclePose& obstacle_pose, seven_robotics_msgs::ObstacleVelocity& obstacle_velocity) //TODO: Calculate projections of obstacles and publish them.
{
    double time_step = 0.5;
    geometry_msgs::Pose obstacle_projected;

    for (int i = 0; i < num_steps; i++)
    {
        obstacle_projected.position.x = obstacle_pose.position.position.x + (obstacle_velocity.relative_velocity.linear.x * (i * time_interval));
        obstacle_projected.position.y = obstacle_pose.position.position.y + (obstacle_velocity.relative_velocity.linear.y * (i * time_interval));
        obstacle_projected.position.z = obstacle_pose.position.position.z + (obstacle_velocity.relative_velocity.linear.z * (i * time_interval));
        obs_pose_array.poses.push_back(obstacle_projected);
    }
    obs_pose_array.header.stamp = ros::Time::now();
    obs_pose_array.header.frame_id = "map";
    obstacle_poseArray_pub_.publish(obs_pose_array);
}

void imminentCollisionAction::odomCB(const nav_msgs::Odometry& odom)
{
    current_robot_vel_ = odom.twist.twist;

    ego_velocity.header.stamp = ros::Time::now();
    ego_velocity.velocity = current_robot_vel_;
    ego_vel_pub_.publish(ego_velocity);
}

void imminentCollisionAction::fadeObstacles()
{
    for(auto& obstacle: dynamic_obstacle_)
    {
        // ROS_INFO("Inside fadeObstacles - for loop.");
        obstacle.second.fade_counter_--;
        ROS_INFO("[IMMINENT COLLISION] obstacle: %d, fade counter: %d", obstacle.first, obstacle.second.fade_counter_);
    }

    // for(auto& dynObstacle: dynObsQueue)
    // {
    //     // ROS_INFO("Inside fadeObstacles - for loop.");
    //     dynObstacle.second.fade_counter_--;
    //     ROS_INFO("[IMMINENT COLLISION](queue) obstacle: %d, fade counter: %d", dynObstacle.second.id_, dynObstacle.second.fade_counter_);
    // }
}

void imminentCollisionAction::removeStaleObstacles()
{
    ROS_INFO("Inside removeStaleObstacles");
    for(auto it = dynamic_obstacle_.begin(); it != dynamic_obstacle_.end();)
    {
        // ROS_INFO("Inside removeStaleObstacles - for loop.");
        // ROS_INFO("[IMMINENT COLLISION] map size: %d", dynamic_obstacle_.size());
        if(it->second.fade_counter_ > 0)
        {
            ROS_INFO("Inside removeStaleObstacles - for loop if.");
            ++it;
        }
        else if (!dynamic_obstacle_.empty())
        {
            ROS_INFO("Inside removeStaleObstacles - for loop else.");
            removeObstacleById((it++)->first);
            ROS_INFO("Inside removeStaleObstacles - for loop else. After Obstacle Removal.");
        }
    }
}

void imminentCollisionAction::obstacleCB(const costmap_converter::ObstacleArrayMsg& msg)
{
    for(int i = 0; i < msg.obstacles.size(); i++)
    {
        double speed = std::sqrt((msg.obstacles[i].velocities.twist.linear.x * msg.obstacles[i].velocities.twist.linear.x) + 
                        (msg.obstacles[i].velocities.twist.linear.y * msg.obstacles[i].velocities.twist.linear.y));
        if(!dynamic_obstacle_.contains(msg.obstacles[i].id) && speed > 0.2)
        {
            costmap_converter::ObstacleMsg obs_msg_ = msg.obstacles[i];
            dynamicObstacle obstacle_;
            obstacle_.id_ = msg.obstacles[i].id;
            obstacle_.pos_.x = obs_msg_.polygon.points[0].x;
            obstacle_.pos_.y = obs_msg_.polygon.points[0].y;
            obstacle_.radius_ = obs_msg_.radius;
            obstacle_.velocity_ = obs_msg_.velocities.twist;
            obstacle_.fade_counter_ = 11;
            dynamic_obstacle_.insert({msg.obstacles[i].id, obstacle_});
        }
        else if(speed > 0.2) 
        {
            costmap_converter::ObstacleMsg obs_msg_ = msg.obstacles[i];
            // dynamicObstacle temp;
            // dynamic_obstacle_.insert(msg.obstacles[i].id,temp);
            dynamic_obstacle_[msg.obstacles[i].id].id_ = obs_msg_.id;
            dynamic_obstacle_[msg.obstacles[i].id].pos_.x = obs_msg_.polygon.points[0].x;
            dynamic_obstacle_[msg.obstacles[i].id].pos_.y = obs_msg_.polygon.points[0].y;
            dynamic_obstacle_[msg.obstacles[i].id].radius_ = obs_msg_.radius;
            dynamic_obstacle_[msg.obstacles[i].id].velocity_ = obs_msg_.velocities.twist;
            dynamic_obstacle_[msg.obstacles[i].id].fade_counter_ = 11;
        }
    }
}

void imminentCollisionAction::egoPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& ego_pose_)
{
    ego_pose = ego_pose_;
}

void imminentCollisionAction::insertObstacle(double distance, const dynamicObstacle& obs)
{
    auto result = dynamic_obstacle_.insert({obs.id_, obs});
    
    if (result.second)//Check if insertion was successful
    { 
        ROS_ERROR("In multiset insert.");
        distance_set.insert({distance, obs.id_});
    } 
    else //Handle duplicates or update existing entries.
    {
        ROS_ERROR("In multiset insert else.");
        for (auto set_it = distance_set.begin(); set_it != distance_set.end(); ++set_it) {
            ROS_ERROR("In multiset insert else for.");
            if (set_it->second == obs.id_) {
                distance_set.erase(set_it);
                break;
            }
        }
        distance_set.insert({distance, obs.id_});
        auto iter_1 = dynamic_obstacle_.find(obs.id_);
        iter_1->second.pos_ = obs.pos_;
        iter_1->second.velocity_ = obs.velocity_;
        iter_1->second.radius_ = obs.radius_;
    }
}

void imminentCollisionAction::removeObstacleById(int id)
{
    ROS_WARN("In remove obstacle.");
    auto it = dynamic_obstacle_.find(id);
    ROS_WARN("In remove obstacle. Before if.");
    if (it != dynamic_obstacle_.end()) {
        ROS_WARN("In remove obstacle if.");
        //To find and remove the corresponding entry in the distance_set.
        for (auto set_it = distance_set.begin(); set_it != distance_set.end(); ++set_it) {
            ROS_WARN("In remove obstacle if for.");
            if (set_it->second == id) {
                ROS_WARN("In remove obstacle if for if.");
                distance_set.erase(set_it);
                ROS_WARN("In remove obstacle if for if. After erase.");
                break;
            }
        }
        //Remove the obstacle from map.
        dynamic_obstacle_.erase(it);
    }
    else
        ROS_ERROR("Did not find the ID %d", id);
}

double imminentCollisionAction::cpDistanceIntersection(geometry_msgs::Pose& ego_pose_, geometry_msgs::Twist& ego_velocity_, geometry_msgs::Pose& obs_pose_, geometry_msgs::Twist& obs_velocity_)
{
    using Line2d = Eigen::Hyperplane<float,2>;
    using Vec2d  = Eigen::Vector2f;

    double v_ego_resultant = std::sqrt((ego_velocity_.linear.x * ego_velocity_.linear.x) + (ego_velocity_.linear.y * ego_velocity_.linear.y));
    double v_obs_resultant = std::sqrt((obs_velocity_.linear.x * obs_velocity_.linear.x) + (obs_velocity_.linear.y * obs_velocity_.linear.y));

    double v_ego_x_norm = (ego_velocity_.linear.x)/v_ego_resultant;
    double v_ego_y_norm = (ego_velocity_.linear.y)/v_ego_resultant;
    double v_obs_x_norm = (obs_velocity_.linear.x)/v_obs_resultant;
    double v_obs_y_norm = (obs_velocity_.linear.y)/v_obs_resultant;

    double delta_x_ego = v_ego_x_norm * 7.0;
    double delta_y_ego = v_ego_y_norm * 7.0;
    double delta_x_obs = v_obs_x_norm * 7.0;
    double delta_y_obs = v_obs_y_norm * 7.0; 

    Vec2d a(ego_pose_.position.x, ego_pose_.position.y);
    Vec2d b(obs_pose_.position.x, obs_pose_.position.y);
    Vec2d c(ego_pose_.position.x + delta_x_ego, ego_pose_.position.y + delta_y_ego);
    Vec2d d(obs_pose_.position.x + delta_x_obs, obs_pose_.position.y + delta_y_obs);

    Line2d ac = Line2d::Through(a,c);
    Line2d bd = Line2d::Through(b,d);

    auto intersect_pt = ac.intersection(bd);

    double distance = std::sqrt(((ego_pose_.position.x - intersect_pt[0]) * (ego_pose_.position.x - intersect_pt[0])) + ((ego_pose_.position.y - intersect_pt[1]) * (ego_pose_.position.y - intersect_pt[1])));

    return distance;
}

void imminentCollisionAction::tick()
{
    //Remove old obstacles from Queue.
    fadeObstacles();
    removeStaleObstacles();

    //Setting FCL object for this robot.
    fcl::Sphere<double> robot_bound (ego_radius);
    std::shared_ptr<fcl::Sphere<double>> robot_sphere_ptr = std::make_shared<fcl::Sphere<double>>(robot_bound);
    fcl::CollisionObject<double> robot_object(robot_sphere_ptr);

    std::vector<int> return_values;

    for (dynObsIter it = dynamic_obstacle_.begin(); it != dynamic_obstacle_.end(); ++it)
    {
        obs_velocity.object_id = it->first;
        obs_velocity.relative_velocity = it->second.velocity_;
        obs_velocity.header.frame_id = "map";
        obs_velocity.header.stamp = ros::Time::now();

        obs_radius.object_id = it->first;
        obs_radius.radius = it->second.radius_;
        obs_radius.header.frame_id = "map";
        obs_radius.header.stamp = ros::Time::now();

        obs_pose.object_id = it->first;
        obs_pose.header.frame_id = "map";
        obs_pose.header.stamp = ros::Time::now();
        obs_pose.position.position.x = it->second.pos_.x;
        obs_pose.position.position.y = it->second.pos_.y;

        if(std::abs(calculateRelativeVel(obs_velocity, ego_velocity)) > 0.2)
        {
            double dist_collide = cpDistanceIntersection(ego_pose.pose.pose, ego_velocity.velocity, obs_pose.position, obs_velocity.relative_velocity);
            if (dist_collide < 50.0 && dist_collide > 0.0){
                ROS_INFO("dist_collide = %f", dist_collide);
                insertObstacle(dist_collide, it->second);
            }
            else
                continue;
        }
    }

    auto current_time = ros::Time::now();

    for (auto it_1 = distance_set.begin(); it_1 != distance_set.end(); ++it_1)
    {
        auto it_2 = dynamic_obstacle_.find(it_1->second);
        double obstacle_radius = it_2->second.radius_;
        ROS_ERROR("[IMMINENT COLLISION] After radius. Radius = %f", obstacle_radius);
        double dist_to_collide = it_1->first;
        ROS_ERROR("[IMMINENT COLLISION] After distance to collision. dist_to_collide = %f", dist_to_collide);

        if((!is_cancel_control_.data) && (dist_to_collide <= 5.0))
        {
            ROS_WARN("[IMMINENT COLLISION] Collision detected at distance %f!", dist_to_collide);
            
            collision_path_ = path_;
            collision_path_pub_.publish(collision_path_);

            is_cancel_control_.data = true;
            should_pause_.publish(is_cancel_control_);
            
            return_values.push_back(1);
            // dynamic_obstacle_.clear();
            // dynObsQueue.clear_all();
        }
        else if((is_cancel_control_.data) && (dist_to_collide <= 5.0))
        {
            ROS_WARN("[IMMINENT COLLISION] STILL IN COLLISION PATH!!!");
            // ros::Duration(5.1).sleep(); //wait for sometime.
            is_cancel_control_.data = true;
            should_pause_.publish(is_cancel_control_);
            // return_values.push_back(100);
            // dynamic_obstacle_.clear();
            // dynObsQueue.clear_all();
        }
        else if((dist_to_collide > 5.0) || ((ros::Time::now() - current_time).toSec() > 30.0))        
        {
            ROS_ERROR("[IMMINENT COLLISION] This obstacle is not in collision path.");
            return_values.push_back(0);
        }
    }

    auto return_value_ = std::accumulate(return_values.begin(), return_values.end(), 0);
    if (return_value_ == 0){
        ROS_INFO("[IMMINENT COLLISION] No obstacles in collision path.");
        is_cancel_control_.data = false;
        should_pause_.publish(is_cancel_control_);
        // dynamic_obstacle_.clear();
        // dynObsQueue.clear_all();
    }

    // dynamic_obstacle_.clear();
    // dynObsQueue.clear_all();
    is_cancel_control_.data = false;
    should_pause_.publish(is_cancel_control_);
}