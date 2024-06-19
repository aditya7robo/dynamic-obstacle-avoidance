#include <seven_bt_plugins/plugins/condition/imminent_collision_action.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <seven_robot_utils/tf_util.hpp>
#include <seven_robot_utils/geometry_utils.hpp>
#include <costmap_2d/cost_values.h>
#include <cmath>
#include <numeric>
#include <message_filters/time_synchronizer.h>
#include <vector>



imminentCollisionAction::imminentCollisionAction(const std::string& xml_tag, const BT::NodeConfig& config)
      : BT::ConditionNode(xml_tag,config)
{
    tf = config.blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
    nh_ = config.blackboard->get<std::shared_ptr<ros::NodeHandle>>("node_handle");
    
    odom_sub_ = nh_->subscribe("/odometry/filtered", 1,&imminentCollisionAction::odomCB,this,ros::TransportHints().tcpNoDelay());
    obstacle_sub_ = nh_->subscribe("/move_base_flex/TebLocalPlannerROS/obstacles",1,&imminentCollisionAction::obstacleCB,this,ros::TransportHints().tcpNoDelay());
    ego_pose_sub_ = nh_->subscribe("/amcl_pose", 1, &imminentCollisionAction::egoPoseCallback, this, ros::TransportHints().tcpNoDelay());

    // obstacle_distance_sub.subscribe(*nh_,"obs_dist",1);
    // obstacle_rel_vel_sub.subscribe(*nh_,"obs_vel_rel",1);
    // ego_vel_sub.subscribe(*nh_,"robot_velocity",1);
    // obstacle_rad_sub.subscribe(*nh_,"obstacles/radius",1);
    // obstacle_pose_sub.subscribe(*nh_, "obs_pose", 1);
    // sync.reset(new SynchronizerType(MySyncPolicy(10), obstacle_distance_sub, obstacle_rel_vel_sub, ego_vel_sub, obstacle_rad_sub, obstacle_pose_sub));
    // sync->registerCallback(boost::bind(&imminentCollisionAction::syncCallback, this, _1, _2, _3, _4, _5));
    
    obstacle_poseArray_pub_ = nh_->advertise<geometry_msgs::PoseArray>("obs_pose_array", 1);
    obstacle_projection_pub_ = nh_->advertise<geometry_msgs::PoseStamped>("obs_projection", 1);
    ego_projection_pub_ = nh_->advertise<geometry_msgs::PoseStamped>("robot_projection", 1);
    
    ego_vel_pub_ = nh_->advertise<seven_robotics_msgs::EgoVelocity>("robot_velocity", 1);

    refined_path_pub_ = nh_->advertise<nav_msgs::Path>("refined_path", 1);
    collision_path_pub_ = nh_->advertise<nav_msgs::Path>("collision_path", 1);

    config.blackboard->set<bool>(is_cancel_control, is_cancel_control_);
}


// void imminentCollisionAction::syncCallback(seven_robotics_msgs::Distance::ConstPtr obstacle_distance_,seven_robotics_msgs::ObstacleVelocity::ConstPtr obstacle_velocity_, seven_robotics_msgs::EgoVelocity::ConstPtr ego_velocity_, seven_robotics_msgs::ObstacleRadius::ConstPtr obstacle_radius_, seven_robotics_msgs::ObstaclePose::ConstPtr obstacle_pose_)
// { 
//     // ROS_ERROR("!!!!!!!!!!!!!!! IMMINENT COLLISION syncCALLBACK =============");
//     obs_distance = *obstacle_distance_;
//     // ROS_ERROR("!!!!!!!!!!!!!!!! obs_dist - %f ==============", obs_distance.distance);
//     obs_velocity = *obstacle_velocity_;
//     // ROS_ERROR("!!!!!!!!!!!!!!!! obs_vel - %f ==============", obs_velocity.relative_velocity);
//     ego_velocity = *ego_velocity_;
//     // ROS_ERROR("!!!!!!!!!!!!!!!! ego_vel - %f ==============", ego_velocity.velocity);
//     obs_radius = *obstacle_radius_;
//     // ROS_ERROR("!!!!!!!!!!!!!!!! obs_rad - %f ==============", obs_radius.radius);
//     obs_pose = *obstacle_pose_;
// }

double imminentCollisionAction::calculateRelativeVel(seven_robotics_msgs::ObstacleVelocity& obstacle_velocity, seven_robotics_msgs::EgoVelocity& ego_velocity)
{
    ROS_ERROR("!!!!!!!!!!!!!!! [IMMINENT COLLISION] CALCULATING RELATIVE VELOCITY !!!!!!!!!!!!!!!");
    double x = obstacle_velocity.relative_velocity.linear.x - ego_velocity.velocity.linear.x;
    double y = obstacle_velocity.relative_velocity.linear.y - ego_velocity.velocity.linear.y;
    double z = obstacle_velocity.relative_velocity.linear.z - ego_velocity.velocity.linear.z;
    double resultant_vel = sqrt((x*x) + (y*y) + (z*z));
    ROS_ERROR("!!!!!!!!!!!!!!! [IMMINENT COLLISION] Relative Vel: %f !!!!!!!!!!!!!!!", resultant_vel);
    ROS_ERROR("!!!!!!!!!!!!!!! [IMMINENT COLLISION] v_x = %f !!!!!!!!!!!!!!!", x);
    ROS_ERROR("!!!!!!!!!!!!!!! [IMMINENT COLLISION] v_y = %f !!!!!!!!!!!!!!!", y);
    
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
        ROS_ERROR("[IMMINENT COLLISION] obstacle: %d, fade counter: %d", obstacle.first, obstacle.second.fade_counter_);
    }

    for(auto& dynObstacle: dynObsQueue)
    {
        // ROS_INFO("Inside fadeObstacles - for loop.");
        dynObstacle.second.fade_counter_--;
        ROS_ERROR("[IMMINENT COLLISION](queue) obstacle: %d, fade counter: %d", dynObstacle.second.id_, dynObstacle.second.fade_counter_);
    }
}

void imminentCollisionAction::removeStaleObstacles()
{
    for(dynObsIter it = dynamic_obstacle_.begin(); it != dynamic_obstacle_.end();)
    {
        // ROS_INFO("Inside removeStaleObstacles - for loop.");
        // ROS_ERROR("[IMMINENT COLLISION] map size: %d", dynamic_obstacle_.size());
        if(it->second.fade_counter_ > 0)
        {
            ROS_INFO("Inside removeStaleObstacles - for loop if.");
            ++it;
        }
        else if (!dynamic_obstacle_.empty())
        {
            ROS_ERROR("[IMMINENT COLLISION] Before Erasing %d", it->first);
            dynamic_obstacle_.erase(it++);
            ROS_ERROR("[IMMINENT COLLISION] After Erasing %d", it->first);
            // ++it;
            // ROS_INFO("Inside removeStaleObstacles - for loop else.");
        }
    }

    for(auto it = dynObsQueue.begin(); it != dynObsQueue.end();)
    {
        ROS_INFO("Inside removeStaleObstacles - for loop.");
        // ROS_ERROR("[IMMINENT COLLISION] map size: %d", dynamic_obstacle_.size());
        if(it->second.fade_counter_ > 0)
        {
            ROS_INFO("Inside removeStaleObstacles - for loop if.");
            ++it;
        }
        else if (!dynObsQueue.empty())
        {
            ROS_ERROR("[IMMINENT COLLISION] Queue Before Erasing %f", it->first);
            dynObsQueue.erase(it++);
            ROS_ERROR("[IMMINENT COLLISION] Queue After Erasing %f", it->first);
            // ++it;
            // ROS_INFO("Inside removeStaleObstacles - for loop else.");
        }
    }
    // ROS_INFO("Inside removeStaleObstacles - after for loop.");
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
    // ego_pose.header.stamp = ros::Time::now();
    // ego_pose.header.frame_id = ego_pose_.header.frame_id;
    // // ego_pose.pose.covariance = ego_pose_.pose.covariance;
    // // std::copy(ego_pose_.pose.covariance.begin(), ego_pose_.pose.covariance.end(), ego_pose.pose.covariance.begin());
    // for (int i = 0; i < ego_pose_.pose.covariance.size(); i++)
    //     ego_pose.pose.covariance[i] = ego_pose_.pose.covariance[i];
    // ego_pose.pose.pose.orientation.w = ego_pose_.pose.pose.orientation.w;
    // ego_pose.pose.pose.orientation.x = ego_pose_.pose.pose.orientation.x;
    // ego_pose.pose.pose.orientation.y = ego_pose_.pose.pose.orientation.y;
    // ego_pose.pose.pose.orientation.z = ego_pose_.pose.pose.orientation.z;
    // ego_pose.pose.pose.position.x = ego_pose_.pose.pose.position.x;
    // ego_pose.pose.pose.position.y = ego_pose_.pose.pose.position.y;
    // ego_pose.pose.pose.position.z = ego_pose_.pose.pose.position.z;
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

BT::NodeStatus imminentCollisionAction::tick()
{
    // if(!synchronizer_.initialized)
    // {
    //     synchronizer_.initialize();
    // }
    ROS_ERROR("!!!!!!!!!!!!!!! [IMMINENT COLLISION] ACTION !!!!!!!!!!!!!!!");

    fadeObstacles();
    ROS_ERROR("[IMMINENT COLLISION] FADE OBSTACLES DONE");
    removeStaleObstacles();
    ROS_ERROR("[IMMINENT COLLISION] REMOVE STALE OBSTACLES DONE");

    getInput("path",goal_.path);
    path_ = goal_.path;

    nav_msgs::Path empty_path;

    // double time_to_collide = 100.0;
    fcl::Sphere<double> robot_bound (ego_radius);
    std::shared_ptr<fcl::Sphere<double>> robot_sphere_ptr = std::make_shared<fcl::Sphere<double>>(robot_bound);
    fcl::CollisionObject<double> robot_object(robot_sphere_ptr);

    if(status() == BT::NodeStatus::IDLE)
    {
        ROS_ERROR("!!!!!!!!!!!!!!! [IMMINENT COLLISION] STATUS RUNNING !!!!!!!!!!!!!!!");
        setStatus(BT::NodeStatus::RUNNING);
    } 
 
    // if(std::abs(calculateRelativeVel(obs_velocity, ego_velocity)) > 0.2)
            
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
            ROS_ERROR("[IMMINENT COLLISION] distance to collide: %f", dist_collide);
            dynObsDistance.first = dist_collide;
            dynObsDistance.second = it->second;
            
            int element_exists_flag = 0;
        
            for (auto it_ = dynObsQueue.begin(); it_ != dynObsQueue.end(); ++it_) {
                if (it_->second.id_ == dynObsDistance.second.id_){
                    element_exists_flag = 1;
                    it_->first = dist_collide;
                    it_->second.pos_ = obs_pose.position.position;
                    it_->second.velocity_ = obs_velocity.relative_velocity;
                    it_->second.radius_ = obs_radius.radius;
                    break;
                }
            }

            if (!element_exists_flag && (dynObsDistance.first < 7.0))
            {       
                dynObsQueue.emplace(dynObsDistance);
            }
        }
    }
    
    auto current_time = ros::Time::now();

    for (auto it = dynObsQueue.begin(); it != dynObsQueue.end(); ++it)
    {
        bool should_cancel_ = config().blackboard->get<bool>("is_cancel_control");
        ROS_ERROR ("[IMMINENT COLLISION](first) is_cancel_control = %d", should_cancel_);

        // if(std::abs(calculateRelativeVel(obs_velocity, ego_velocity)) > 0.2)
        // {
        // double time_to_collide;
        // time_to_collide = obs_distance_/(ego_velocity_ - obs_velocity_);

        // geometry_msgs::Twist obstacle_velocity_ = it->second.velocity_;

        ROS_ERROR("!!!!!!!!!!!!!!! [IMMINENT COLLISION] FOUND 1 COLLISION PROBABLY !!!!!!!!!!!!!!!");
        
        double obstacle_radius = it->second.radius_;
        fcl::Sphere<double> obstacle_bound(obstacle_radius);
        std::shared_ptr<fcl::Sphere<double>> obs_sphere_ptr = std::make_shared<fcl::Sphere<double>>(obstacle_bound);
        fcl::CollisionObject<double> obs_object(obs_sphere_ptr);

        // imminentCollisionAction::projectPoses(obs_pose, obs_velocity);

        // int num_steps = 6; //Number of Time Steps.
        // float time_interval = 0.5; //Time interval over which to check for collisions.
        for(int step = 0; step < num_steps; ++step)
        {
            // ROS_ERROR("[IMMINENT COLLISION] step %d", step+1);
            // if (step >= 5)
            //     return BT::NodeStatus::RUNNING;
            // else
            //     return BT::NodeStatus::FAILURE;
            // ROS_ERROR("!!!!!!!!!!!!!!! IMMINENT COLLISION CALCULATING !!!!!!!!!!!!!!!");

            fcl::DynamicAABBTreeCollisionManager<double> collision_manager;

            collision_manager.registerObject(&robot_object);
            collision_manager.registerObject(&obs_object);

            geometry_msgs::Twist obstacle_velocity_ = it->second.velocity_;
            fcl::Vector3d obstacle_translation(it->second.pos_.x + ((obstacle_velocity_.linear.x)*time_interval*(step+1)), it->second.pos_.y + ((obstacle_velocity_.linear.y)*time_interval*(step+1)), 0.0);
            obs_object.setTranslation(obstacle_translation);

            obs_projected_pose.pose.position.x = obstacle_translation[0];
            obs_projected_pose.pose.position.y = obstacle_translation[1];
            obs_projected_pose.header.stamp = ros::Time::now();
            obs_projected_pose.header.frame_id = "map";
            obstacle_projection_pub_.publish(obs_projected_pose);

            fcl::Vector3d robot_translation(ego_pose.pose.pose.position.x + ((ego_velocity.velocity.linear.x)*time_interval*(step+1)), ego_pose.pose.pose.position.y + ((ego_velocity.velocity.linear.y)*time_interval*(step+1)), 0.0);
            robot_object.setTranslation(robot_translation);

            ego_projected_pose.pose.position.x = robot_translation[0];
            ego_projected_pose.pose.position.y = robot_translation[1];
            ego_projected_pose.header.stamp = ros::Time::now();
            ego_projected_pose.header.frame_id = "map";
            ego_projection_pub_.publish(ego_projected_pose); 

            fcl::CollisionRequestd request;
            fcl::CollisionResultd result;

            fcl::DistanceRequestd request_distance;
            fcl::DistanceResultd result_distance;
    
            fcl::distance(&robot_object, &obs_object, request_distance, result_distance);
            double minimum_distance = result_distance.min_distance;
            ROS_ERROR("[IMMINENT COLLISION] fcl cp distance: %f", result_distance.min_distance);

            fcl::collide(&robot_object, &obs_object, request, result);
            // collision_manager.collide(&robot_object, &obs_object, &request, &result);

            
            // getInput("is_cancel_control", is_cancel_control_);
            // bool should_cancel_ = config().blackboard->get<bool>("is_cancel_control");
            // ROS_ERROR ("[IMMINENT COLLISION] is_cancel_control = %d", should_cancel_);

            if((result.isCollision() && !should_cancel_) || ((minimum_distance <= 1.5) && !should_cancel_))
            {
                ROS_ERROR("[IMMINENT COLLISION] Collision detected at step %d", step+1);
                
                collision_path_ = path_;
                collision_path_pub_.publish(collision_path_);
                
                return_values.push_back(1);
                return BT::NodeStatus::SUCCESS;
            }
            else if((result.isCollision() && should_cancel_) || ((minimum_distance <= 1.5) && should_cancel_))
            {
                ROS_ERROR("!!!!!!!!!!!!!!! [IMMINENT COLLISION] STILL IN COLLISION PATH !!!!!!!!!!!!!!!");
                // ros::Duration(5.1).sleep(); //wait for sometime.
                return_values.push_back(100);
                return BT::NodeStatus::RUNNING;
            }
            else if((!result.isCollision() || ((ros::Time::now() - current_time).toSec() > 30.0)) && (minimum_distance > 1.5))
            {
                ROS_ERROR("!!!!!!!!!!!!!!! [IMMINENT COLLISION] OUT OF COLLISION PATH !!!!!!!!!!!!!!!");
                // ROS_ERROR("[IMMINENT COLLISION] Time passed: %f", (ros::Time::now() - current_time).toSec());
                // is_cancel_control_ = false;
                // config().blackboard->set<bool>(is_cancel_control, is_cancel_control_);
                // ROS_ERROR ("[IMMINENT COLLISION](Second) is_cancel_control = %d", is_cancel_control_);
                // ros::Duration(5.1).sleep();

                // if (!collision_path_.poses.empty()){
                //     empty_path.poses.clear();
                //     setOutput("refined_path", collision_path_);
                // }
                // else{
                //     empty_path.poses.clear();
                //     setOutput("refined_path", empty_path);
                // }

                // setOutput("refined_path", path_);
                refined_path_pub_.publish(path_);

                return_values.push_back(0);
                // return BT::NodeStatus::FAILURE;
            }
        }
        // }
        // else
        // {
        //     is_cancel_control_ = false;
        //     config().blackboard->set<bool>(is_cancel_control, is_cancel_control_);
        //     ROS_ERROR ("[IMMINENT COLLISION](Third) is_cancel_control = %d", is_cancel_control_);
        //     return_values.push_back(0);
        //     // return BT::NodeStatus::FAILURE;
        // }
    }

    auto return_value_ = std::accumulate(return_values.begin(), return_values.end(), 0);
    ROS_ERROR("[IMMINENT COLLISION] return_values = %d", return_value_);
    if (return_value_ == 0){
        ROS_ERROR("[IMMINENT COLLISION] No obstacles in collision path.");
        is_cancel_control_ = false;
        config().blackboard->set<bool>(is_cancel_control, is_cancel_control_);
        ROS_ERROR("[IMMINENT COLLISION](Fourth) is_cancel_control = %d", is_cancel_control_);
        // if (!collision_path_.poses.empty()){
        //     empty_path.poses.clear();
        //     setOutput("refined_path", collision_path_);
        // }
        // else{
        //     setOutput("refined_path", path_);
        // }
        setOutput("refined_path", path_);
        return BT::NodeStatus::FAILURE;
    }
    // else if ((return_value_ < 100) && (return_value_ != 0)){
    //     is_cancel_control = true;
    //     return BT::NodeStatus::SUCCESS;
    // }
    // else
    //     return BT::NodeStatus::RUNNING;

    return BT::NodeStatus::FAILURE;
}

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<imminentCollisionAction>("ImminentCollisionAction");
}