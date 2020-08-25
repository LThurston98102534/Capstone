#include "tracker.h"

/*
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sstream>
*/

Tracker::Tracker(ros::NodeHandle nh)
    : nh_(nh), listener_(new(tf::TransformListener)), broadcaster_(new(tf::TransformBroadcaster)), refresh_rate_(default_refresh_rate)
{
    sub1_ = nh_.subscribe("ar_pose_marker", 1000, &Tracker::trackerCallback, this);


}

Tracker::~Tracker()
{

}


void Tracker::trackerCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
{
    // Create unique lock for safe access to member variables
    std::unique_lock<std::mutex> marker_pose_lock(marker_pose_mutex_);

    // Update member variable with most recent pose data
    marker_pose_ = *msg;

    marker_pose_lock.unlock();
    
}
    


  



void Tracker::trackTags()
{
    // Set refresh rate to required value (10 seconds as per specification)
    ros::Rate rate_limiter(refresh_rate_);
    while (ros::ok()) {
          
          // Create unique lock for safe access to member variables
          std::unique_lock<std::mutex> marker_pose_lock(marker_pose_mutex_);

          // Update member variable with most recent pose data
          marker_pose_for_calc_ = marker_pose_;

          marker_pose_lock.unlock();
          
          geometry_msgs::PoseStamped sensor_pose;
          float timestamp = 0.0;
          tf::Transform transform;
          
          for (int i = 0; i < marker_pose_for_calc_.markers.size(); i++) {
            if (marker_pose_for_calc_.markers.at(i).id == 4) {
                ROS_INFO("I see marker: [%d] which is the fridge!", marker_pose_for_calc_.markers.at(i).id);
                sensor_pose = marker_pose_for_calc_.markers.at(i).pose;
                sensor_pose.header.frame_id = marker_pose_for_calc_.markers.at(i).header.frame_id;
                transform.setOrigin(tf::Vector3(sensor_pose.pose.position.x, sensor_pose.pose.position.y, sensor_pose.pose.position.z));
                transform.setRotation(tf::Quaternion(sensor_pose.pose.orientation.x, sensor_pose.pose.orientation.y, sensor_pose.pose.orientation.z, sensor_pose.pose.orientation.w));
                broadcaster_->sendTransform(tf::StampedTransform(transform, ros::Time::now(), sensor_pose.header.frame_id, "fridge_link"));
                
                
                timestamp = marker_pose_for_calc_.markers.at(i).header.stamp.toSec();
            } else{
                if (marker_pose_for_calc_.markers.at(i).id == 0) {
                    ROS_INFO("I see marker: [%d] which is the sealer!", marker_pose_for_calc_.markers.at(i).id);
                    sensor_pose = marker_pose_for_calc_.markers.at(i).pose;
                    sensor_pose.header.frame_id = marker_pose_for_calc_.markers.at(i).header.frame_id;
                    transform.setOrigin(tf::Vector3(sensor_pose.pose.position.x, sensor_pose.pose.position.y, sensor_pose.pose.position.z));
                transform.setRotation(tf::Quaternion(sensor_pose.pose.orientation.x, sensor_pose.pose.orientation.y, sensor_pose.pose.orientation.z, sensor_pose.pose.orientation.w));
                broadcaster_->sendTransform(tf::StampedTransform(transform, ros::Time::now(), sensor_pose.header.frame_id, "sealer_link"));
                    
                    timestamp = marker_pose_for_calc_.markers.at(i).header.stamp.toSec();
                } else {
                    ROS_INFO("Not sure what I can see!");
                }
            }
            
              try{
                geometry_msgs::PoseStamped base_pose;
                
                listener_->transformPose("base_link", sensor_pose, base_pose);

                ROS_INFO("sensor_pose Position: (%.2f, %.2f. %.2f) \nsensor_pose Orientation: (%.2f, %.2f, %.2f, %.2f) ----->\nbase_link Position: (%.2f, %.2f, %.2f)\nbase_link Orientation: (%.2f, %.2f, %.2f, %.2f) \nat time %.2f",
                    sensor_pose.pose.position.x, sensor_pose.pose.position.y, sensor_pose.pose.position.z, sensor_pose.pose.orientation.x, sensor_pose.pose.orientation.y, sensor_pose.pose.orientation.z, sensor_pose.pose.orientation.w,            base_pose.pose.position.x, base_pose.pose.position.y, base_pose.pose.position.z, base_pose.pose.orientation.x, base_pose.pose.orientation.y, base_pose.pose.orientation.z, base_pose.pose.orientation.w, timestamp);
              }
              catch(tf::TransformException& ex){
                ROS_ERROR("Received an exception trying to transform a pose from \"sensor_pose\" to \"base_link\": %s", ex.what());
              }
                
          }

       // Delay thread for remainder of time in 10 second loop
       rate_limiter.sleep();

    }




}
