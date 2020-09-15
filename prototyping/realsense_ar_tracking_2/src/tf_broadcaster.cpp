#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;

  while(n.ok()){
    //broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.428, -0.031, 0.2)),ros::Time::now(),"robot_base_link", "base_link"));
          
    
          
    broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.625, 0.0, 0.15)),ros::Time::now(),"robot_base_link", "arm"));

    broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(-0.7071068, 0, 0, 0.7071068), tf::Vector3(0.0, 0.034, 0.1018)),ros::Time::now(),"arm", "calib_tag"));
              


    r.sleep();
  }
}
