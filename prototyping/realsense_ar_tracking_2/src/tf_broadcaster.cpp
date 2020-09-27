#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;

  while(n.ok()){
    //broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.428, -0.031, 0.2)),ros::Time::now(),"robot_base_link", "base_link"));
          
    
          
  //  broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(1, 0, 0, 0), tf::Vector3(1.325, 0.0, 0.15)),ros::Time::now(),"robot_base_link", "end_effector"));

   // broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0.6335811, -0.6335811, 0.4440158), tf::Vector3(0.034, 0, 0.1218)),ros::Time::now(),"r6_gripper", "calib_tag"));
              


    r.sleep();
  }
}
