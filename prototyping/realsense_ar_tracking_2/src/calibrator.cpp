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

/*
    marker_pose_for_calc_ = *msg;

    geometry_msgs::PoseStamped sensor_pose;
    float timestamp = 0.0;
    tf::Transform camera_tag_transform;
          
    for (int i = 0; i < marker_pose_for_calc_.markers.size(); i++) {
        if (marker_pose_for_calc_.markers.at(i).id == 8) {
            ROS_INFO("I see the calibration marker!");
	    sensor_pose = marker_pose_for_calc_.markers.at(i).pose;
	    sensor_pose.header.frame_id = marker_pose_for_calc_.markers.at(i).header.frame_id;

		    
	    camera_tag_transform.setOrigin(tf::Vector3(sensor_pose.pose.position.x, sensor_pose.pose.position.y, sensor_pose.pose.position.z));
            camera_tag_transform.setRotation(tf::Quaternion(sensor_pose.pose.orientation.x, sensor_pose.pose.orientation.y, sensor_pose.pose.orientation.z, sensor_pose.pose.orientation.w));

	    // Get position of Arm w.r.t. tag from kinematic chain
	    tf::StampedTransform tag_arm_transform;
	    listener_->lookupTransform("arm", "calib_tag", ros::Time(0), tag_arm_transform);



	    // Get position of Base w.r.t. arm from kinematic chain
	    tf::StampedTransform arm_base_transform;
	    listener_->lookupTransform("robot_base_link", "arm", ros::Time(0), arm_base_transform);


	    // Calculate relative transform between base and camera by solving the equation X = (A*B*C)^-1*I
	    tf::Transform calc_transform;
	    calc_transform.mult(camera_tag_transform, tag_arm_transform);
	    calc_transform *= arm_base_transform;

	    tf::Transform base_camera_transform;
	    tf::Transform identity_transform;
	    identity_transform.getIdentity();

	    base_camera_transform = calc_transform.inverseTimes(identity_transform);

	    tf::Matrix3x3 base_camera_rotation = base_camera_transform.getBasis();
	    tf::Vector3 base_camera_translation = base_camera_transform.getOrigin();

 	    broadcaster_->sendTransform(tf::StampedTransform(base_camera_transform, ros::Time::now(), "robot_base_link", "base_link"));

	    
        }

    }

*/


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
        if(calib_pos_count_ < 5) {
            
            // Ask user what position to drive arm to
            std::cout << "Select which Arm Position to Drive KUKA Arm to: ";
            while(!(std::cin >> arm_position_)){
                std::cin.clear();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                std::cout << "Invalid input, Try again: ";
            }
           
                   
            std::cout << "Driving Robot Arm to Position: " << arm_position_ << std::endl;     
            // Drive robot arm to position
            
	    //**********************************************************************//
	    //*		NEED TO WORK OUT HOW TO IMPLEMENT THE ARM DRIVE FUNCTION   *//
	    //**********************************************************************//
            
            
            
            
            
            // Prompt user to hit enter once robot arm has stopped moving
            std::cout << "Press Space Bar then Enter when Robot Arm Stops Moving to Continue Calibration... ";
	    std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), ' ');
            


            std::cout << "Do the calibration thing" << std::endl;
            // Get position of tag w.r.t. camera from Alvar package
	    // Create unique lock for safe access to member variables
            std::unique_lock<std::mutex> marker_pose_lock(marker_pose_mutex_);

            // Update member variable with most recent pose data
            marker_pose_for_calc_ = marker_pose_;

            marker_pose_lock.unlock();

            geometry_msgs::PoseStamped sensor_pose;
            float timestamp = 0.0;
	    tf::Transform camera_tag_transform;
          
            for (int i = 0; i < marker_pose_for_calc_.markers.size(); i++) {
                if (marker_pose_for_calc_.markers.at(i).id == 8) {
                    ROS_INFO("I see the calibration marker!");
		    sensor_pose = marker_pose_for_calc_.markers.at(i).pose;
		    sensor_pose.header.frame_id = marker_pose_for_calc_.markers.at(i).header.frame_id;

		    
		    camera_tag_transform.setOrigin(tf::Vector3(sensor_pose.pose.position.x, sensor_pose.pose.position.y, sensor_pose.pose.position.z));
                    camera_tag_transform.setRotation(tf::Quaternion(sensor_pose.pose.orientation.x, sensor_pose.pose.orientation.y, sensor_pose.pose.orientation.z, sensor_pose.pose.orientation.w));



		    // Get position of Arm w.r.t. tag from kinematic chain
		    //**********************************************************************//
	  	    //*		NEED TO GET THE ACTUAL VALUES OF THESE TRANSFORMS          *//
		    //*		NEED TO WORK OUT HOW TO INTEGRATE THIS WITH EXISTING URDF  *//
	  	    //**********************************************************************//

	    	    tf::StampedTransform tag_arm_transform;
	    	    listener_->lookupTransform("arm", "calib_tag", ros::Time(0), tag_arm_transform);
		


	   	    // Get position of Base w.r.t. arm from kinematic chain
		    //**********************************************************************//
	  	    //*		NEED TO GET THE ACTUAL VALUES OF THESE TRANSFORMS          *//
		    //*		NEED TO WORK OUT HOW TO INTEGRATE THIS WITH EXISTING URDF  *//
	  	    //**********************************************************************//

	    	    tf::StampedTransform arm_base_transform;
	    	    listener_->lookupTransform("robot_base_link", "arm", ros::Time(0), arm_base_transform);



	            // Calculate relative transform between base and camera by solving the equation X = (A*B*C)^-1*I
		    tf::Transform calc_transform;
		    calc_transform.mult(camera_tag_transform, tag_arm_transform);
		    calc_transform *= arm_base_transform;

		    tf::Transform base_camera_transform;
		    tf::Transform identity_transform;
		    identity_transform.getIdentity();

		    base_camera_transform = calc_transform.inverseTimes(identity_transform);

		    tf::Matrix3x3 base_camera_rotation = base_camera_transform.getBasis();
		    tf::Vector3 base_camera_translation = base_camera_transform.getOrigin();

		    broadcaster_->sendTransform(tf::StampedTransform(base_camera_transform, ros::Time::now(), "robot_base_link", "base_link"));




		    // DEBUGGING PRINTING STATEMENTS
		    std::cout << std::endl;
		    std::cout << std::endl;
		    std::cout << "Base to Camera Transform: " << std::endl;
		    std::cout << base_camera_rotation[0][0] << " " <<  base_camera_rotation[0][1] << " " << base_camera_rotation[0][2] << " " << base_camera_translation[0] << std::endl;
		    std::cout << base_camera_rotation[1][0] << " " <<  base_camera_rotation[1][1] << " " << base_camera_rotation[1][2] << " " << base_camera_translation[1] << std::endl;
		    std::cout << base_camera_rotation[2][0] << " " <<  base_camera_rotation[2][1] << " " << base_camera_rotation[2][2] << " " << base_camera_translation[2] << std::endl;

		    std::cout << std::endl;
		    std::cout << std::endl;

		    tf::Quaternion base_camera_rotation_quat = base_camera_transform.getRotation();
		    std::cout << "Base to Camera Quaternion: " << std::endl;
		    std::cout << base_camera_rotation_quat[0] << " " <<  base_camera_rotation_quat[1] << " " << base_camera_rotation_quat[2] << " " << base_camera_rotation_quat[3] << std::endl;


		    calib_pos_count_++;

                } else{
		    ROS_INFO("Camera cannot see the calibration marker!");

		}
	    
	    }
        
            
        } else {
	    // Calculate the average difference between the expected and observed
            // Publish this difference to the terminal
    
            std::cout << "Averaging Results" << std::endl;


	}
       
    
    }

       

}
