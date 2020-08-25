#include "buffer.h"
#include <set>
#include <thread>

ROSDataBuffer::ROSDataBuffer(ros::NodeHandle nh, const std::shared_ptr<FrontierExplorer> & frontier_explorer_passed)
    : nh_(nh), it_(nh), explorer_(frontier_explorer_passed), configure_parameters_(false), refresh_rate_(default_refresh_rate)
{
    //Subscribing to odometry
    sub1_ = nh_.subscribe("odom", 1000, &ROSDataBuffer::odomCallback,this);
    //Subscribing to laser
    sub2_ = nh_.subscribe("base_scan_1", 10, &ROSDataBuffer::laserCallback,this);

    //Subscribing to image
    image_transport::ImageTransport it(nh);
    sub3_ = it.subscribe("map_image/full", 1, &ROSDataBuffer::imageCallback,this);

    //Publishing an image to required topic
    image_pub_ = it_.advertise("map_image/fbe", 1);

    // Set up service client
    request_path_client_ = nh_.serviceClient<a5_setup::RequestGoal>("request_goal");

    //Below is how to get parameters from command line, on command line they need to be _param:=value
    ros::NodeHandle pn("~");
    pn.param<double>("resolution", resolution_, default_map_resolution);

    // Initialise Frontier Cells vector of vectors
    frontier_cells_.resize(0);

}

ROSDataBuffer::~ROSDataBuffer()
{
    cv::destroyWindow("view");
}



// A callback for odometry
void ROSDataBuffer::odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
    // Create unique lock for safe access to member variables
    std::unique_lock<std::mutex> robot_pose_lock(pose_mutex_);

    // Update member variable with most recent pose data
    robot_pose_ = msg->pose.pose;

    robot_pose_lock.unlock();

}



void ROSDataBuffer::laserCallback(const sensor_msgs::LaserScanConstPtr& msg)
{

   // Create unique lock for safe access to member variables
   std::unique_lock<std::mutex> laser_lock(laser_parameters_mutex_);

   // If it is the first time laserCallback is running, store laser parameters in member variable and set flag to true
   if(!configure_parameters_){
       robot_laser_.laser_max_range_ = msg->range_max;
       robot_laser_.laser_max_angle_ = msg->angle_max;
       robot_laser_.laser_min_angle_ = msg->angle_min;
       robot_laser_.laser_resolution_ = msg->angle_increment;

       configure_parameters_ = true;

       // Notify all threads waiting for this condition variable
       cv_parameters_.notify_all();

   }

   laser_lock.unlock();

}

void ROSDataBuffer::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    // Below code converts from sensor message to a pointer to an opencv image, to share across threads
    try
    {
      if (enc::isColor(msg->encoding))
        cvPtr_ = cv_bridge::toCvCopy(msg, enc::BGR8);
      else
        cvPtr_ = cv_bridge::toCvCopy(msg, enc::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }



    // Create unique lock for safe access to member variables
    std::unique_lock<std::mutex> image_lock(image_mutex_);

    // Update member variable with latest grayscale OG Map image
    image_ = cvPtr_->image;

    image_lock.unlock();

}


void ROSDataBuffer::publishImage(cv_bridge::CvImage & passed_image){

    // Define image encoding and header
    passed_image.encoding = "bgr8";
    passed_image.header = std_msgs::Header();


    // We now publish the image and use the inbuilt ros function to convert cv_image to an image message
    image_pub_.publish(passed_image.toImageMsg());
}



void ROSDataBuffer::requestService(geometry_msgs::Pose goal_pose_global){

    // Declare request variable and assign values of global goal pose
    a5_setup::RequestGoal srv_request;
    srv_request.request.x = goal_pose_global.position.x;
    srv_request.request.y = goal_pose_global.position.y;
    srv_request.request.theta = tf::getYaw(goal_pose_global.orientation);


    // Call service and if successful publish a new OG Map image to the topic
    if(request_path_client_.call(srv_request)){

        // Create unique lock for safe access to member variables
        std::unique_lock<std::mutex> fresh_image_lock(image_mutex_);

        // Declare new image and convert latest image data to an RGB image
        cv_bridge::CvImage fresh_image;
        cv::cvtColor(image_,fresh_image.image, CV_GRAY2RGB);

        fresh_image_lock.unlock();

        // Publish image to topic
        publishImage(fresh_image);

    }
}


void ROSDataBuffer::exploreUnknown() {
   /**
    * The below loop runs until ros is shutdown, to ensure this thread does not remain
    * a zombie thread
    *
    */

    // Create unique lock for safe access to member variables
    std::unique_lock<std::mutex> parameters_lock(laser_parameters_mutex_);

    // Wait until the laserCallback has been run at least once, to ensure laser parameters have been captured
    while(!configure_parameters_){
        cv_parameters_.wait(parameters_lock);
    }

    // Set laser parameters and resolution in the FrontierExplorer object
    explorer_->setParameters(robot_laser_, resolution_);

    parameters_lock.unlock();


    // Set refresh rate to required value (10 seconds as per specification)
    ros::Rate rate_limiter(refresh_rate_);
    while (ros::ok()) {

        // Create unique lock for safe access to member variables
        std::unique_lock<std::mutex> parameters_lock(image_mutex_);

        // As long as the image contains data, execute the following commands
        if((!image_.empty())){

            // Safely obtain parameter data from member variables
            cv::Mat parameter_image = image_;

            geometry_msgs::Pose parameter_robot_pose = robot_pose_;

            parameters_lock.unlock();

            // Call Frontier Based Exploration function in FrontierExplorer class
            cv_bridge::CvImage cv_image = explorer_->generateGoalPose(parameter_image, parameter_robot_pose);

            // Publish the updated image showing frontiers, goal pose and frontiers seen from goal pose
            publishImage(cv_image);

            // Store grouped frontier cells in member variable
            frontier_cells_.resize(explorer_->getFrontiers().size());
            frontier_cells_ = explorer_->getFrontiers();

            // Request Goal Pose in the request goal service
            requestService(explorer_->getGoalPose());

        } else{
            parameters_lock.unlock();
        }

       // Delay thread for remainder of time in 10 second loop
       rate_limiter.sleep();

    }


}


