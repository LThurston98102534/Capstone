#include <ros/ros.h>
#include "calibrator.h"


int main(int argc, char** argv){
  // Initialise ROS Node
  ros::init(argc, argv, "sealer_fridge_calibrator");

  // ROS Node Handle
  ros::NodeHandle nh;

  // Create Calibrator Object
  std::shared_ptr<Calibrator> calibrator(new Calibrator(nh));

  // Calibration Thread
  std::thread t(&Calibrator::calibrateSealerFridge, calibrator);


  ros::spin();

  ros::shutdown();

  t.join();

  return 0;
}
