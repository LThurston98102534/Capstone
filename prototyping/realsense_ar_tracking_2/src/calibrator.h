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

};







#endif // CALIBRATOR_H
