#ifndef SVM_PIPELINE_HPP
#define SVM_PIPELINE_HPP

#include <ros/ros.h>
#include "shared/as_lib/common.h"
#include "common_msgs/ConeDetections.h"
#include "common_msgs/Cone.h"
#include "nav_msgs/Path.h"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/PoseStamped.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/ml.hpp>
#include <cv_bridge/cv_bridge.h>
class SVM {
public:
  // Constructor:
  SVM();

  // Getters:
  nav_msgs::Path const & getPath() const;
  sensor_msgs::Image const & getSvmImage() const;

  // Changes path information to the new value received:
  void setConeDetectionsSlam(const common_msgs::ConeDetections &newConeDetections);
  void setConeDetectionsSensorFusion(const common_msgs::ConeDetections &newConeDetections);

  void runAlgorithm();
  
private:
  common_msgs::ConeDetections _coneDetections;
  nav_msgs::Path _centerLine;
  nav_msgs::Path _lastPath;
  sensor_msgs::Image _svmImage;
  int _mission;

  cv::Ptr<cv::ml::SVM> _svmModel;
  
  void findCenterLine();
  void setupModel();
};

#endif 
