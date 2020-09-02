#include "calibrator.h"


Calibrator::Calibrator(ros::NodeHandle nh)
    : nh_(nh), listener_(new(tf::TransformListener)), broadcaster_(new(tf::TransformBroadcaster)), refresh_rate_(default_refresh_rate), calib_pos_count_(0), arm_position_(1)
{
    sub1_ = nh_.subscribe("ar_pose_marker", 1000, &Calibrator::trackerCallback, this);


}

Calibrator::~Calibrator()
{

}


void Calibrator::trackerCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
{
    // Create unique lock for safe access to member variables
    std::unique_lock<std::mutex> marker_pose_lock(marker_pose_mutex_);

    // Update member variable with most recent pose data
    marker_pose_ = *msg;

    marker_pose_lock.unlock();
    
}
    


 

void Calibrator::calibrateCamera()
{
    while(ros::ok()) {
        // While position count < 5
        while(calib_pos_count_ < 5) {
            
            // Ask user what position to drive arm to
            std::cout << "Select which Arm Position to Drive KUKA Arm to: ";
            while(!(std::cin >> arm_position_)){
                std::cin.clear();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                std::cout << "Invalid input, Try again: ";
            }
            
                   
            std::cout << "Driving Robot Arm to Position: " << arm_position_ << std::endl;     
            // Drive robot arm to position
            
            
            
            
            
            
            // Prompt user to hit enter once robot arm has stopped moving
            std::cout << "Press Enter when Robot Arm Stops Moving to Continue Calibration... " << std::endl;
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            
            std::cout << "Do the calibration thing" << std::endl;
                // Get position of tag w.r.t. camera from kinematic chain - tag_expected
                // Measure observed position of tag w.r.t. camera from Alvar package - tag_observed
                // Calculate relative transform between tag_expected and tag_observed to obtain the difference in these positions
        
        
        
        
        
            calib_pos_count_++;
        }

                
        // Calculate the average difference between the expected and observed
        // Publish this difference to the terminal
    
        std::cout << "Averaging Results" << std::endl;
    
    
    
    }

        

















/*

    // Set refresh rate to default value
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
                broadcaster_->sendTransform(tf::StampedTransform(transform, ros::Time::now(), sensor_pose.header.frame_id, "fridge_tag"));
                
                broadcaster_->sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),ros::Time::now(),"fridge_tag", "fridge_link"));
                
                
                timestamp = marker_pose_for_calc_.markers.at(i).header.stamp.toSec();
            } else{
                if (marker_pose_for_calc_.markers.at(i).id == 0) {
                    ROS_INFO("I see marker: [%d] which is the sealer!", marker_pose_for_calc_.markers.at(i).id);
                    
                    
                    sensor_pose = marker_pose_for_calc_.markers.at(i).pose;
                    sensor_pose.header.frame_id = marker_pose_for_calc_.markers.at(i).header.frame_id;
                    transform.setOrigin(tf::Vector3(sensor_pose.pose.position.x, sensor_pose.pose.position.y, sensor_pose.pose.position.z));
                    transform.setRotation(tf::Quaternion(sensor_pose.pose.orientation.x, sensor_pose.pose.orientation.y, sensor_pose.pose.orientation.z, sensor_pose.pose.orientation.w));
                    broadcaster_->sendTransform(tf::StampedTransform(transform, ros::Time::now(), sensor_pose.header.frame_id, "sealer_tag"));
                    
                    broadcaster_->sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),ros::Time::now(),"sealer_tag", "sealer_link"));
                    
                    timestamp = marker_pose_for_calc_.markers.at(i).header.stamp.toSec();
                } else {
                    ROS_INFO("Not sure what I can see!");
                }
            }
            
              try{
                geometry_msgs::PoseStamped base_pose;
                
                listener_->transformPose("robot_base_link", sensor_pose, base_pose);



                ROS_INFO("sensor_pose Position: (%.2f, %.2f. %.2f) \nsensor_pose Orientation: (%.2f, %.2f, %.2f, %.2f) ----->\nbase_link Position: (%.2f, %.2f, %.2f)\nbase_link Orientation: (%.2f, %.2f, %.2f, %.2f) \nat time %.2f",
                    sensor_pose.pose.position.x, sensor_pose.pose.position.y, sensor_pose.pose.position.z, sensor_pose.pose.orientation.x, sensor_pose.pose.orientation.y, sensor_pose.pose.orientation.z, sensor_pose.pose.orientation.w,            base_pose.pose.position.x, base_pose.pose.position.y, base_pose.pose.position.z, base_pose.pose.orientation.x, base_pose.pose.orientation.y, base_pose.pose.orientation.z, base_pose.pose.orientation.w, timestamp);
                    
                    
                    
              }
              catch(tf::TransformException& ex){
                ROS_ERROR("Received an exception trying to transform a pose from \"sensor_pose\" to \"base_link\": %s", ex.what());
              }
                
          }

       // Delay thread for remainder of time in loop
       rate_limiter.sleep();

    }

*/


}
