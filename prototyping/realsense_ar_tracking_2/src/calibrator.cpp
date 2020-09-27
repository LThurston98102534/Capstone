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
        if(calib_pos_count_ < 5) {
            

/*
            // Ask user what position to drive arm to
            std::cout << "Select which Arm Position to Drive KUKA Arm to: ";
            while(!(std::cin >> arm_position_)){
                std::cin.clear();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                std::cout << "Invalid input, Try again: ";
            }
           
  */                 
            std::cout << "Driving Robot Arm to Position: 1" << arm_position_ << std::endl;     
            // Drive robot arm to position
            
	    //**********************************************************************//
	    //*		NEED TO WORK OUT HOW TO IMPLEMENT THE ARM DRIVE FUNCTION   *//
	    //**********************************************************************//
            
/*            
            tf::Transform Tab;
	    tf::Transform Tbc;
	    tf::Transform Tca;

	    Tab.setOrigin(tf::Vector3(-2, 2, 1));
            Tab.setRotation(tf::Quaternion(1, 0, 0, 0));

	    Tbc.setOrigin(tf::Vector3(-2, 4, 1));
            Tbc.setRotation(tf::Quaternion(1, 0, 0, 0));

	    Tca.mult(Tbc.inverse(), Tab.inverse());



	    tf::Transform base_camera_transform;

	    base_camera_transform = Tca; //.inverseTimes(identity_transform);

	    tf::Matrix3x3 base_camera_rotation = base_camera_transform.getBasis();
	    tf::Vector3 base_camera_translation = base_camera_transform.getOrigin();


	    // DEBUGGING PRINTING STATEMENTS
	    std::cout << std::endl;
	    std::cout << std::endl;
	    std::cout << "Using Aron's Formula: " << std::endl;
	    std::cout << "Tca: " << std::endl;
	    std::cout << base_camera_rotation[0][0] << " " <<  base_camera_rotation[0][1] << " " << base_camera_rotation[0][2] << " " << base_camera_translation[0] << std::endl;
	    std::cout << base_camera_rotation[1][0] << " " <<  base_camera_rotation[1][1] << " " << base_camera_rotation[1][2] << " " << base_camera_translation[1] << std::endl;
	    std::cout << base_camera_rotation[2][0] << " " <<  base_camera_rotation[2][1] << " " << base_camera_rotation[2][2] << " " << base_camera_translation[2] << std::endl;

	    std::cout << std::endl;
	    std::cout << std::endl;

	    tf::Quaternion base_camera_rotation_quat = base_camera_transform.getRotation();
	    std::cout << "Tca Quaternion: " << std::endl;
	    std::cout << base_camera_rotation_quat[0] << " " <<  base_camera_rotation_quat[1] << " " << base_camera_rotation_quat[2] << " " << base_camera_rotation_quat[3] << std::endl;


            std::cout << std::endl;
	    std::cout << std::endl;
	    std::cout << std::endl;
	    std::cout << std::endl;







	    Tca.mult(Tab, Tbc);


	    tf::Transform identity_transform;
	    identity_transform = identity_transform.getIdentity();

	    base_camera_transform = Tca.inverseTimes(identity_transform);

	    base_camera_rotation = base_camera_transform.getBasis();
	    base_camera_translation = base_camera_transform.getOrigin();


	    // DEBUGGING PRINTING STATEMENTS
	    std::cout << std::endl;
	    std::cout << std::endl;
	    std::cout << "Using Liam's Formula: " << std::endl;
	    std::cout << "Tca: " << std::endl;
	    std::cout << base_camera_rotation[0][0] << " " <<  base_camera_rotation[0][1] << " " << base_camera_rotation[0][2] << " " << base_camera_translation[0] << std::endl;
	    std::cout << base_camera_rotation[1][0] << " " <<  base_camera_rotation[1][1] << " " << base_camera_rotation[1][2] << " " << base_camera_translation[1] << std::endl;
	    std::cout << base_camera_rotation[2][0] << " " <<  base_camera_rotation[2][1] << " " << base_camera_rotation[2][2] << " " << base_camera_translation[2] << std::endl;

	    std::cout << std::endl;
	    std::cout << std::endl;

	    base_camera_rotation_quat = base_camera_transform.getRotation();
	    std::cout << "Tca Quaternion: " << std::endl;
	    std::cout << base_camera_rotation_quat[0] << " " <<  base_camera_rotation_quat[1] << " " << base_camera_rotation_quat[2] << " " << base_camera_rotation_quat[3] << std::endl;


            std::cout << std::endl;
	    std::cout << std::endl;
	    std::cout << std::endl;
	    std::cout << std::endl;

*/


            
            // Prompt user to hit enter once robot arm has stopped moving
            std::cout << "Press Space Bar then Enter when Robot Arm Stops Moving to Continue Calibration... ";
	    std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), ' ');
            


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

    
		    tf::StampedTransform camera_internal_transform;
		    listener_->lookupTransform("camera_base_link", sensor_pose.header.frame_id, ros::Time(0), camera_internal_transform);
	    	    //listener_->lookupTransform(sensor_pose.header.frame_id, "camera_base_link", ros::Time(0), camera_internal_transform);

		    tf::Matrix3x3 camera_internal_rotation = camera_internal_transform.getBasis();
	   	    tf::Vector3 camera_internal_translation = camera_internal_transform.getOrigin();

		    std::cout << std::endl;
		    std::cout << std::endl;
		    std::cout << "Camera's Internal Transform: " << std::endl;
		    std::cout << camera_internal_rotation[0][0] << " " <<  camera_internal_rotation[0][1] << " " << camera_internal_rotation[0][2] << " " << camera_internal_translation[0] << std::endl;
		    std::cout << camera_internal_rotation[1][0] << " " <<  camera_internal_rotation[1][1] << " " << camera_internal_rotation[1][2] << " " << camera_internal_translation[1] << std::endl;
		    std::cout << camera_internal_rotation[2][0] << " " <<  camera_internal_rotation[2][1] << " " << camera_internal_rotation[2][2] << " " << camera_internal_translation[2] << std::endl;

		    std::cout << std::endl;
		    std::cout << std::endl;

		    tf::Quaternion camera_internal_quat = camera_internal_transform.getRotation();
		    std::cout << "Camera's Internal Quaternion: " << std::endl;
		    std::cout << camera_internal_quat[0] << " " <<  camera_internal_quat[1] << " " << camera_internal_quat[2] << " " << camera_internal_quat[3] << std::endl;


		    std::cout << std::endl;
		    std::cout << std::endl;
		    std::cout << std::endl;
		    std::cout << std::endl;


		    // Get position of End Effector w.r.t. tag from kinematic chain
		    //**********************************************************************//
	  	    //*		NEED TO GET THE ACTUAL VALUES OF THESE TRANSFORMS          *//
		    //*		NEED TO WORK OUT HOW TO INTEGRATE THIS WITH EXISTING URDF  *//
	  	    //**********************************************************************//
/*
	    	    tf::StampedTransform tag_arm_transform;
	    	    listener_->lookupTransform("link_7", "tool0", ros::Time(0), tag_arm_transform);
		


	   	    // Get position of Base w.r.t. End Effector from kinematic chain
		    //**********************************************************************/
	  	    //*		NEED TO GET THE ACTUAL VALUES OF THESE TRANSFORMS          *//
		    //*		NEED TO WORK OUT HOW TO INTEGRATE THIS WITH EXISTING URDF  *//
	  	    //**********************************************************************//
/*
	    	    tf::StampedTransform arm_base_transform;
	    	    listener_->lookupTransform("base_link", "link_7", ros::Time(0), arm_base_transform);


                     

	            // Calculate relative transform between base and camera by solving the equation X = (A*B*C)^-1*I
		    tf::Transform calc_transform;
	  	    calc_transform.mult(camera_internal_transform, camera_tag_transform);
		    calc_transform = calc_transform.inverse();
                    calc_transform *= tag_arm_transform;
		    calc_transform *= arm_base_transform;

		    tf::Transform base_camera_transform;
		    tf::Transform identity_transform;
		    identity_transform.getIdentity();

		    base_camera_transform = calc_transform; //.inverseTimes(identity_transform);

		    tf::Matrix3x3 base_camera_rotation = base_camera_transform.getBasis();
		    tf::Vector3 base_camera_translation = base_camera_transform.getOrigin();

		    broadcaster_->sendTransform(tf::StampedTransform(base_camera_transform, ros::Time::now(), "base_link", "camera_base_link"));

*/

		    tf::StampedTransform tag_arm_transform;
	    	    listener_->lookupTransform("link_7", "tool0", ros::Time(0), tag_arm_transform);
		


	   	    // Get position of Base w.r.t. End Effector from kinematic chain
		    //**********************************************************************//
	  	    //*		NEED TO GET THE ACTUAL VALUES OF THESE TRANSFORMS          *//
		    //*		NEED TO WORK OUT HOW TO INTEGRATE THIS WITH EXISTING URDF  *//
	  	    //**********************************************************************//

	    	    tf::StampedTransform arm_base1_transform;
	    	    listener_->lookupTransform("base_link", "link_1", ros::Time(0), arm_base1_transform);

		    tf::StampedTransform arm_base2_transform;
	    	    listener_->lookupTransform("link_1", "link_2", ros::Time(0), arm_base2_transform);
		    
		    tf::StampedTransform arm_base3_transform;
	    	    listener_->lookupTransform("link_2", "link_3", ros::Time(0), arm_base3_transform);

		    tf::StampedTransform arm_base4_transform;
	    	    listener_->lookupTransform("link_3", "link_4", ros::Time(0), arm_base4_transform);

		    tf::StampedTransform arm_base5_transform;
	    	    listener_->lookupTransform("link_4", "link_5", ros::Time(0), arm_base5_transform);

		    tf::StampedTransform arm_base6_transform;
	    	    listener_->lookupTransform("link_5", "link_6", ros::Time(0), arm_base6_transform);

		    tf::StampedTransform arm_base7_transform;
	    	    listener_->lookupTransform("link_6", "link_7", ros::Time(0), arm_base7_transform);



                     

	            // Calculate relative transform between base and camera by solving the equation X = (A*B*C)^-1*I
		    tf::Transform calc_transform;
	  	    calc_transform.mult(arm_base1_transform, arm_base2_transform);
		    calc_transform *= arm_base3_transform;
		    calc_transform *= arm_base4_transform;
		    calc_transform *= arm_base5_transform;
		    calc_transform *= arm_base6_transform;
		    calc_transform *= arm_base7_transform;
		    calc_transform *= tag_arm_transform;
   

		    tf::Transform temp_transform;
	 	    temp_transform.mult(camera_internal_transform, camera_tag_transform);

	            temp_transform = temp_transform.inverse();

                    calc_transform *= temp_transform;

		    tf::Transform base_camera_transform;
		    tf::Transform identity_transform;
		    identity_transform.getIdentity();

		    base_camera_transform = calc_transform; //.inverseTimes(identity_transform);

		    tf::Matrix3x3 base_camera_rotation = base_camera_transform.getBasis();
		    tf::Vector3 base_camera_translation = base_camera_transform.getOrigin();

		    broadcaster_->sendTransform(tf::StampedTransform(base_camera_transform, ros::Time::now(), "base_link", "camera_base_link"));





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
	    calib_pos_count_ = 0;


	}
       
    
    }

}




void Calibrator::calibrateSealerFridge()
{
    while(ros::ok()) {
        // While position count < 5
        if(calib_pos_count_ < 5) {
            
            // Tell user to position KUKA Arm in required position
            std::cout << "Position KUKA Arm in required deposit position" << std::endl;
            
            
            // Prompt user to hit enter once robot is in position
            std::cout << "Press Space Bar then Enter when ready to continue Calibration... ";
	    std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), ' ');
            


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
                if (marker_pose_for_calc_.markers.at(i).id == 0 || marker_pose_for_calc_.markers.at(i).id == 4) {
                    ROS_INFO("I see the marker!");
		    sensor_pose = marker_pose_for_calc_.markers.at(i).pose;
		    sensor_pose.header.frame_id = marker_pose_for_calc_.markers.at(i).header.frame_id;

		    
		    camera_tag_transform.setOrigin(tf::Vector3(sensor_pose.pose.position.x, sensor_pose.pose.position.y, sensor_pose.pose.position.z));
                    camera_tag_transform.setRotation(tf::Quaternion(sensor_pose.pose.orientation.x, sensor_pose.pose.orientation.y, sensor_pose.pose.orientation.z, sensor_pose.pose.orientation.w));

		


	   	    // Get position of Base w.r.t. arm from kinematic chain
		    //**********************************************************************//
	  	    //*		NEED TO GET THE ACTUAL VALUES OF THESE TRANSFORMS          *//
		    //*		NEED TO WORK OUT HOW TO INTEGRATE THIS WITH EXISTING URDF  *//
	  	    //**********************************************************************//

	    	    tf::StampedTransform arm_base_transform;
	    	    listener_->lookupTransform("robot_base_link", "arm", ros::Time(0), arm_base_transform);



		    // Get position of camera w.r.t. robot base from kinematic chain
		    //**********************************************************************//
	  	    //*		NEED TO GET THE ACTUAL VALUES OF THESE TRANSFORMS          *//
		    //*		NEED TO WORK OUT HOW TO INTEGRATE THIS WITH EXISTING URDF  *//
	  	    //**********************************************************************//

	    	    tf::StampedTransform base_camera_transform;
	    	    listener_->lookupTransform("camera", "robot_base_link", ros::Time(0), base_camera_transform);


                    // 

	            // Calculate relative transform between sealer/fridge tag and target position by solving the equation X = (A*B*C)^-1*I
		    tf::Transform calc_transform;
		    calc_transform.mult(arm_base_transform, base_camera_transform);
		    calc_transform *= camera_tag_transform;

		    tf::Transform tag_target_transform;
		    tf::Transform identity_transform;
		    identity_transform.getIdentity();

		    tag_target_transform = calc_transform.inverseTimes(identity_transform);

		    tf::Matrix3x3 tag_target_rotation = tag_target_transform.getBasis();
		    tf::Vector3 tag_target_translation = tag_target_transform.getOrigin();


		    if (marker_pose_for_calc_.markers.at(i).id == 0) {
			// Sealer Unit
		        broadcaster_->sendTransform(tf::StampedTransform(tag_target_transform, ros::Time::now(), "sealer_tag", "sealer_link"));

		    } else {
			// Fridge
		        broadcaster_->sendTransform(tf::StampedTransform(tag_target_transform, ros::Time::now(), "fridge_tag", "fridge_link"));

		    }

		    // DEBUGGING PRINTING STATEMENTS
		    std::cout << std::endl;
		    std::cout << std::endl;
		    std::cout << "AR Tag to Target Transform: " << std::endl;
		    std::cout << tag_target_rotation[0][0] << " " <<  tag_target_rotation[0][1] << " " << tag_target_rotation[0][2] << " " << tag_target_translation[0] << std::endl;
		    std::cout << tag_target_rotation[1][0] << " " <<  tag_target_rotation[1][1] << " " << tag_target_rotation[1][2] << " " << tag_target_translation[1] << std::endl;
		    std::cout << tag_target_rotation[2][0] << " " <<  tag_target_rotation[2][1] << " " << tag_target_rotation[2][2] << " " << tag_target_translation[2] << std::endl;

		    std::cout << std::endl;
		    std::cout << std::endl;

		    tf::Quaternion tag_target_rotation_quat = tag_target_transform.getRotation();
		    std::cout << "AR Tag to Target Quaternion: " << std::endl;
		    std::cout << tag_target_rotation_quat[0] << " " <<  tag_target_rotation_quat[1] << " " << tag_target_rotation_quat[2] << " " << tag_target_rotation_quat[3] << std::endl;


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
