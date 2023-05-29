#ifndef SVM_HANDLE_HPP
#define SVM_HANDLE_HPP

#include "svm/svm_pipeline.hpp"
typedef enum ConeDetections {
  SLAM,
  SENSORFUSION
} ConeDetections;

class SVMHandle {

public: 
  // Constructor
  SVMHandle(ros::NodeHandle &nodeHandle);

  //Methods
  void getConeDetectionsMethod();
  void advertiseToTopics(); 
  void subscribeToTopics();
  void publishToTopics();
  void run();

private: 
  ros::NodeHandle _nodeHandle;
  ros::Publisher _pubPath;
  ros::Publisher _pubSvmImage;
  ros::Subscriber _subConeDetectionsSlam;
  ros::Subscriber _subConeDectionsSensorFusion;

  SVM _svm;
  ConeDetections _coneDetections = SENSORFUSION;

  void coneDetectionsSlamCallback(const common_msgs::ConeDetections &conesDetected);
  void coneDetectionsSensorFusionCallback(const common_msgs::ConeDetections &conesDetected);
};

#endif 