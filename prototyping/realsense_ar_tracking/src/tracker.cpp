#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>


#include <sstream>

tf::TransformListener* listener=NULL;
tf::TransformBroadcaster* br=NULL;


void trackerCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
{
 // tf::TransformListener listener(ros::Duration(10));
  geometry_msgs::PoseStamped sensor_pose;
  float timestamp = 0.0;
  tf::Transform transform;
  
  for (int i = 0; i < msg->markers.size(); i++) {
    if (msg->markers.at(i).id == 4) {
        ROS_INFO("I see marker: [%d] which is the fridge!", msg->markers.at(i).id);
        sensor_pose = msg->markers.at(i).pose;
        sensor_pose.header.frame_id = msg->markers.at(i).header.frame_id;
        transform.setOrigin(tf::Vector3(sensor_pose.pose.position.x, sensor_pose.pose.position.y, sensor_pose.pose.position.z));
        transform.setRotation(tf::Quaternion(sensor_pose.pose.orientation.x, sensor_pose.pose.orientation.y, sensor_pose.pose.orientation.z, sensor_pose.pose.orientation.w));
        br->sendTransform(tf::StampedTransform(transform, ros::Time::now(), sensor_pose.header.frame_id, "fridge_link"));
        
        
        timestamp = msg->markers.at(i).header.stamp.toSec();
    } else{
        if (msg->markers.at(i).id == 0) {
            ROS_INFO("I see marker: [%d] which is the sealer!", msg->markers.at(i).id);
            sensor_pose = msg->markers.at(i).pose;
            sensor_pose.header.frame_id = msg->markers.at(i).header.frame_id;
            transform.setOrigin(tf::Vector3(sensor_pose.pose.position.x, sensor_pose.pose.position.y, sensor_pose.pose.position.z));
        transform.setRotation(tf::Quaternion(sensor_pose.pose.orientation.x, sensor_pose.pose.orientation.y, sensor_pose.pose.orientation.z, sensor_pose.pose.orientation.w));
        br->sendTransform(tf::StampedTransform(transform, ros::Time::now(), sensor_pose.header.frame_id, "sealer_link"));
            
            timestamp = msg->markers.at(i).header.stamp.toSec();
        } else {
            ROS_INFO("Not sure what I can see!");
        }
    }
    
      try{
        geometry_msgs::PoseStamped base_pose;
 //       listener.waitForTransform("base_link", sensor_pose.header.frame_id, ros::Time(0), ros::Duration(3.0));
        
        listener->transformPose("base_link", sensor_pose, base_pose);

        ROS_INFO("sensor_pose Position: (%.2f, %.2f. %.2f) \nsensor_pose Orientation: (%.2f, %.2f, %.2f, %.2f) ----->\nbase_link Position: (%.2f, %.2f, %.2f)\nbase_link Orientation: (%.2f, %.2f, %.2f, %.2f) \nat time %.2f",
            sensor_pose.pose.position.x, sensor_pose.pose.position.y, sensor_pose.pose.position.z, sensor_pose.pose.orientation.x, sensor_pose.pose.orientation.y, sensor_pose.pose.orientation.z, sensor_pose.pose.orientation.w,            base_pose.pose.position.x, base_pose.pose.position.y, base_pose.pose.position.z, base_pose.pose.orientation.x, base_pose.pose.orientation.y, base_pose.pose.orientation.z, base_pose.pose.orientation.w, timestamp);
      }
      catch(tf::TransformException& ex){
        ROS_ERROR("Received an exception trying to transform a pose from \"sensor_pose\" to \"base_link\": %s", ex.what());
      }
        
  }
  
}


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can 
   * perform any ROS arguments and name remapping that were provided at
   * the command line. For programmatic remappings you can use a
   * different version of init() which takes remappings directly, but
   * for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any
   * other part of the ROS system.
   */
  ros::init(argc, argv, "tracker");

  /**
   * NodeHandle is the main access point to communications with the
   * ROS system. The first NodeHandle constructed will fully initialize
   * this node, and the last NodeHandle destructed will close down
   * the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive
   * messages on a given topic.  This invokes a call to the ROS master
   * node, which keeps a registry of who is publishing and who is subscribing.
   * Messages are passed to a callback function, here called chatterCallback.  
   * subscribe() returns a Subscriber object that you must hold on to
   * until you want to unsubscribe. When all copies of the Subscriber
   * object go out of scope, this callback will automatically be
   * unsubscribed from this topic.
   *
   * The second parameter to the subscribe() function is the size of
   * the message queue.  If messages are arriving faster than they are
   * being processed, this is the number of messages that will be
   * buffered up before beginning to throw away the oldest ones.
   */
   
  listener = new(tf::TransformListener);
  br = new(tf::TransformBroadcaster);
  
  ros::Subscriber sub = n.subscribe("ar_pose_marker", 1000, trackerCallback);

    /**
   * ros::spin() will enter a loop, pumping callbacks.  With this
   * version, all callbacks will be called from within this thread
   * (the main one).  ros::spin() will exit when Ctrl-C is pressed,
   * or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
