#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;

  while(n.ok()){
     //broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0.7071068, 0.7071068), tf::Vector3(-0.152, 0.04857, 0.2075)),ros::Time::now(),"base_link", "camera_base_link"));
          
    
          
    //broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(1, 0, 0, 0), tf::Vector3(1.325, 0.0, 0.15)),ros::Time::now(),"base_link", "link_7"));

    //broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0.6335811, -0.6335811, 0.4440158), tf::Vector3(0.034, 0, 0.1218)),ros::Time::now(),"link_7", "tool0"));
              


    r.sleep();
  }
}
