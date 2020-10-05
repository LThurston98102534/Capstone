#ifndef CALIBRATOR_H
#define CALIBRATOR_H

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <sstream>
#include <iostream>
#include <limits>

#include <thread>
#include <chrono>
#include <deque>
#include <mutex>
#include <random>
#include <condition_variable>


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"

const double default_refresh_rate = 5;

class Calibrator{
public:
    Calibrator(ros::NodeHandle nh);

    ~Calibrator();


    void trackerCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);

    void calibrateCamera();

    void calibrateSealerFridge();

    void broadcastBaseCameraTransform();

    void broadcastTagTargetTransform();

    ros::NodeHandle nh_;

private:
    tf::TransformListener* listener_;
    tf::TransformBroadcaster* broadcaster_;

    ros::Subscriber sub1_;

    ar_track_alvar_msgs::AlvarMarkers marker_pose_;
    std::mutex marker_pose_mutex_;
    
    ar_track_alvar_msgs::AlvarMarkers marker_pose_for_calc_;

    double refresh_rate_;
    int calib_pos_count_;
    int arm_position_;
    int transform_set_;
    std::mutex transform_set_mutex_;

    double cumulative_rot_x_;
    double cumulative_rot_y_;
    double cumulative_rot_z_;
    double cumulative_rot_w_;

    double cumulative_trans_x_;
    double cumulative_trans_y_;
    double cumulative_trans_z_;

    tf::Transform average_base_camera_transform_;
    std::mutex base_camera_mutex_;


    tf::Transform average_tag_target_transform_;
    std::mutex tag_target_mutex_;

};







#endif // CALIBRATOR_H
