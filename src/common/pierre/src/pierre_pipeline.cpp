#include <ros/ros.h>
#include <pierre/pierre_pipeline.hpp>

/****************************************
*             General Tasks             *
****************************************/

// Constructor
Pierre::Pierre(): tfListener(tf_buffer_), _pclPointCloud(new PclPointCloud){
}

std::tuple<int,int> Pierre::initParameters() {

    if(!ros::param::get("/common/camera_number", _numberCameras)) {
        ROS_WARN_STREAM("Could not load parameter camera_number. Default will be assumed");
        _numberCameras = 2;
    }
    
    if(!ros::param::get("/perception/pipeline", _pipeline)) {
        ROS_WARN_STREAM("Could not load parameter pipeline. Default will be assumed");
        _pipeline = THESIS;
    }

	if(!ros::param::has("/common/K_32")  &&
       !ros::param::has("/common/K_54")  &&
       !ros::param::has("/common/R_left")  &&
	   !ros::param::has("/common/T_left")  &&
       !ros::param::has("/common/R_right") &&
	   !ros::param::has("/common/T_right")) {
		ROS_ERROR("This node needs the sensor matrices. Please load them.");
		exit(EXIT_FAILURE);	
	}

    /* load left camera parameters */
    std::vector<double> K;
    std::vector<double> R;
    std::vector<double> T;
    ros::param::get("/common/K_54",     K);
    ros::param::get("/common/R_left",   R);
    ros::param::get("/common/T_left",   T);

    _left.K = Eigen::Matrix3d(K.data());
    _left.R = Eigen::Matrix3d(R.data());
    _left.T = Eigen::Vector3d(T.data());
    _left.side = LEFT_CAMERA;

    _left.K.transposeInPlace();
    _left.R.transposeInPlace();
    
    /* load right camera parameters */

    ros::param::get("/common/K_32",     K);
    ros::param::get("/common/R_right",  R);
    ros::param::get("/common/T_right",  T);

    _right.K = Eigen::Matrix3d(K.data());
    _right.R = Eigen::Matrix3d(R.data());
    _right.T = Eigen::Vector3d(T.data());
    _right.side = RIGHT_CAMERA;

    _right.K.transposeInPlace();
    _right.R.transposeInPlace();
    
    return std::make_tuple(_pipeline, _numberCameras);
}

// Getters
sensor_msgs::Image const & Pierre::getVisualization() const { return _visualization; }
sensor_msgs::Image const & Pierre::getLeftCameraVisualization() const { return _leftCameraVisualization; }
sensor_msgs::Image const & Pierre::getRightCameraVisualization() const { return _rightCameraVisualization; }

// Update methods
void Pierre::updateCameraImage(const sensor_msgs::Image &newCameraImage) {
    _cameraImageVector.push_back(newCameraImage);
}

void Pierre::updateConeDetections(const common_msgs::ConeDetections &newConeDetections) {
    _coneDetections = newConeDetections;
    transformConeDetections();
}

void Pierre::updateCoNetProposals(const common_msgs::ConeDetections &newCoNetProposals) {
    _coNetProposals = newCoNetProposals;
}

void Pierre::updateLidarClusteredPoints(const sensor_msgs::PointCloud2 &newLidarClusteredPoints) {
    _lidarClusteredPoints = newLidarClusteredPoints;
}

void Pierre::updateCenterline(const nav_msgs::Path &newCenterline) {
    _centerline = newCenterline;
}

void Pierre::updatePointToFollow(const visualization_msgs::Marker &newPointToFollow) {
    _pointToFollow = newPointToFollow.pose.position;
}

void Pierre::updateConfiguration(int configuration) {
    _configuration = configuration;
}

void Pierre::updatePipelineParameters(const pierre::PierreConfig &config) {
    
   _manualCalibration = config.calibration_manual;

    if(!_manualCalibration) return;
    
    _tx = config.left_tx;
    _ty = config.left_ty;
    _tz = config.left_tz;
    _rx = toRad(config.left_rx);
    _ry = toRad(config.left_ry);
    _rz = toRad(config.left_rz);

    buildMatrices(_left);

    if (_numberCameras != 2) return;

    _tx = config.right_tx;
    _ty = config.right_ty;
    _tz = config.right_tz;
    _rx = toRad(config.right_rx);
    _ry = toRad(config.right_ry);
    _rz = toRad(config.right_rz);

    buildMatrices(_right);
}

void Pierre::buildMatrices(Camera &camera){
    Eigen::Matrix3d Rx, Ry, Rz;

    Rx << 1, 	0, 		0,
            0, cos(_rx), -sin(_rx),
            0, sin(_rx), cos(_rx);

    Ry << cos(_ry), 0, sin(_ry),
            0, 	 1,  	0,
            -sin(_ry), 0, cos(_ry);

    Rz << cos(_rz), -sin(_rz), 0,
            sin(_rz), cos(_rz),  0,
            0, 		  0, 	  1;

    camera.R = Rx*Ry*Rz;
    camera.T = Eigen::Vector3d(_tx, _ty, _tz);
}

double Pierre::toRad(double deg){
    return deg * M_PI / 180;
}

/********  End of General Tasks  ********/


/*************************************************
*                Pierre Processing               *
*************************************************/

void Pierre::runAlgorithm() {
    if(!_cameraImageVector.empty())
        visualizeEverythingOnImage();
    _cameraImageVector.clear();
}

/**
*	Name: pointsConversion.
*	Description: Converts input point to camera coordinates.
*	Inputs: none
*	Output: void
*/
std::vector<Eigen::Vector4d> Pierre::pointsConversion(std::vector<Eigen::Vector4d> points) {

    std::vector<Eigen::Vector4d> convertedPoints;
    Camera camera;
    for (size_t i = 0; i < points.size(); ++i) {
        Eigen::Vector3d point(points[i](0), points[i](1), points[i](2));
        if((_numberCameras == 2 && point(1) > 0) || _numberCameras == 1) {
            camera = _left;
        } else {
            camera = _right;
        }
        point = camera.R * point;
        point += camera.R * camera.T;
        //rotation in z
        point = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX()) * point;
        //rotation in y
        point = Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitY()) * point;
        point = camera.K * point;
        Eigen::Vector4d convertedPoint(point(0)/point(2), point(1)/point(2), camera.side, points[i](3));
        convertedPoints.push_back(convertedPoint);
    }
    return convertedPoints;
}

std::vector<Eigen::Vector3d> Pierre::centerlineConversion() {
	std::vector<Eigen::Vector3d> convertedPoints;
    Camera camera;
    for (size_t i = 0; i < _centerline.poses.size(); ++i) {
        Eigen::Vector3d point(_centerline.poses[i].pose.position.x, _centerline.poses[i].pose.position.y, _centerline.poses[i].pose.position.z-0.25);
        if((_numberCameras == 2 && point(1) > 0) || _numberCameras == 1) {
            camera = _left;
        } else {
            camera = _right;
        }
        point = camera.R * point;
        point += camera.R * camera.T;
        //rotation in z
        point = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX()) * point;
        //rotation in y
        point = Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitY()) * point;
        point = camera.K * point;
        Eigen::Vector3d convertedPoint(point(0)/point(2), point(1)/point(2), camera.side);
        convertedPoints.push_back(convertedPoint);
    }
    
	return convertedPoints;
}

/**
*	Name: projectBoundingBox.
*	Description: Projects (Xmin,Ymin) and (Xmax,Ymax) to create the bounding boxes
*	Inputs: cone msg
*	Output: void
*/
std::vector<std::vector<Eigen::Vector4d>> Pierre::projectBoundingBox(std::vector<Eigen::Vector4d> points) {

    std::vector<std::vector<Eigen::Vector4d>> convertedMinMaxPoints;
    Camera camera;
    for (size_t i = 0; i < points.size(); ++i) {
        Eigen::Vector3d pointMin(points[i](0), points[i](1) + 0.17, points[i](2) + 0.27);
        Eigen::Vector3d pointMax(points[i](0), points[i](1) - 0.17, points[i](2) - 0.17);
        std::vector<Eigen::Vector3d> bbPoints = {pointMin, pointMax};
        std::vector<Eigen::Vector4d> coordinates(bbPoints.size());
        
        for(size_t j = 0; j < bbPoints.size() ; ++j) {
            Eigen::Vector3d point(bbPoints[j](0), bbPoints[j](1), bbPoints[j](2));
            if((_numberCameras == 2 && point(1) > 0) || _numberCameras == 1) {
                camera = _left;
            } else {
                camera = _right;
            }
            point = camera.R * point;
            point += camera.R * camera.T;
            //rotation in z
            point = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX()) * point;
            //rotation in y
            point = Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitY()) * point;
            point = camera.K * point;
            Eigen::Vector4d convertedPoint(point(0)/point(2), point(1)/point(2), camera.side, points[i](3));
            coordinates[j] = convertedPoint;
        }
        convertedMinMaxPoints.push_back(coordinates);
    }
    return convertedMinMaxPoints;
}

/**
*	Name: visualizeClustersOnCamera.
*	Description:
*	Inputs: none
*	Output: void
*/
void Pierre::visualizeEverythingOnImage() {
    // get cv image from image msg
    cv_bridge::CvImagePtr leftImage;
    cv_bridge::CvImagePtr rightImage;

    try {
        if(_numberCameras == 2) {
            leftImage = cv_bridge::toCvCopy(_cameraImageVector[LEFT_CAMERA], sensor_msgs::image_encodings::BGR8);
            rightImage = cv_bridge::toCvCopy(_cameraImageVector[RIGHT_CAMERA], sensor_msgs::image_encodings::BGR8);
        } else {
            leftImage = cv_bridge::toCvCopy(_cameraImageVector[LEFT_CAMERA], sensor_msgs::image_encodings::BGR8);
        }
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
    if(_configuration >= 1) {
        // extract points from pointcloud
        std::vector<Eigen::Vector4d> clusterPoints = extractPointsFromCloud();

        // convert all points
        std::vector<Eigen::Vector4d> convertedClusterPoints = pointsConversion(clusterPoints);

        // Draw clustered points
        for (size_t i = 0; i < convertedClusterPoints.size(); ++i) {
            if(convertedClusterPoints[i](2) == LEFT_CAMERA) {
                cv::circle(leftImage->image, cv::Point(convertedClusterPoints[i](0), convertedClusterPoints[i](1)), 3, CV_RGB(0,0,0), cv::FILLED);
            } else {
                cv::circle(rightImage->image, cv::Point(convertedClusterPoints[i](0), convertedClusterPoints[i](1)), 3, CV_RGB(0,0,0), cv::FILLED);
            }
        }
    }
    
    if(_configuration >= 2) {
        std::vector<Eigen::Vector4d> points;
        for (int i = 0; i < _coNetProposals.cone_detections.size(); i++){
            Eigen::Vector4d point(_coNetProposals.cone_detections[i].position.x, _coNetProposals.cone_detections[i].position.y,
                                     _coNetProposals.cone_detections[i].position.z, _coNetProposals.cone_detections[i].color);
            points.push_back(point);
        }
        // convert centroids
        std::vector<Eigen::Vector4d> coNetCentroids = pointsConversion(points);
        // convert BoundingBox points
        std::vector<std::vector<Eigen::Vector4d>> convertedMinMaxPointsCoNet = projectBoundingBox(points);
        
        for (size_t i = 0; i < convertedMinMaxPointsCoNet.size(); ++i) {
            // Draw centroid
            if(coNetCentroids[i](2) == LEFT_CAMERA) {
                cv::circle(leftImage->image, cv::Point(coNetCentroids[i](0), coNetCentroids[i](1)), 7, CV_RGB(0,255,0), cv::FILLED);
            } else {
                cv::circle(rightImage->image, cv::Point(coNetCentroids[i](0), coNetCentroids[i](1)), 7, CV_RGB(0,255,0), cv::FILLED);
            }
            
            if(_pipeline == THESIS) {
                // Draw bounding box
                if(convertedMinMaxPointsCoNet[i][0](2) == LEFT_CAMERA) {
                    cv::rectangle(leftImage->image, cv::Point(convertedMinMaxPointsCoNet[i][0](0), convertedMinMaxPointsCoNet[i][0](1)), 
                            cv::Point(convertedMinMaxPointsCoNet[i][1](0), convertedMinMaxPointsCoNet[i][1](1)), cv::Scalar(0,0,0), 4);
                } else {
                    cv::rectangle(rightImage->image, cv::Point(convertedMinMaxPointsCoNet[i][0](0), convertedMinMaxPointsCoNet[i][0](1)), 
                            cv::Point(convertedMinMaxPointsCoNet[i][1](0), convertedMinMaxPointsCoNet[i][1](1)), cv::Scalar(0,0,0), 4);
                }
            }
        }

        points.clear();
        for (int i = 0; i < _coneDetections.cone_detections.size(); i++){
            Eigen::Vector4d point(_coneDetections.cone_detections[i].position.x, _coneDetections.cone_detections[i].position.y,
                                     _coneDetections.cone_detections[i].position.z, _coneDetections.cone_detections[i].color);
            points.push_back(point);
        }
        // convert centroids
        std::vector<Eigen::Vector4d> lidarCentroids = pointsConversion(points);
        // convert BoundingBox points
        std::vector<std::vector<Eigen::Vector4d>> convertedMinMaxPoints = projectBoundingBox(points);
        
        for (size_t i = 0; i < convertedMinMaxPoints.size(); ++i) {
            // Draw centroid
            if(lidarCentroids[i](2) == LEFT_CAMERA) {
                cv::circle(leftImage->image, cv::Point(lidarCentroids[i](0), lidarCentroids[i](1)), 7, CV_RGB(0,255,0), cv::FILLED);
            } else {
                cv::circle(rightImage->image, cv::Point(lidarCentroids[i](0), lidarCentroids[i](1)), 7, CV_RGB(0,255,0), cv::FILLED);
            }
            
            if(_pipeline == THESIS) {
                // Draw bounding box
                if(convertedMinMaxPoints[i][0](2) == LEFT_CAMERA) {
                    cv::rectangle(leftImage->image, cv::Point(convertedMinMaxPoints[i][0](0), convertedMinMaxPoints[i][0](1)), 
                            cv::Point(convertedMinMaxPoints[i][1](0), convertedMinMaxPoints[i][1](1)), getBoundingBoxColor(convertedMinMaxPoints[i][1](3)), 4);
                } else {
                    cv::rectangle(rightImage->image, cv::Point(convertedMinMaxPoints[i][0](0), convertedMinMaxPoints[i][0](1)), 
                            cv::Point(convertedMinMaxPoints[i][1](0), convertedMinMaxPoints[i][1](1)), getBoundingBoxColor(convertedMinMaxPoints[i][1](3)), 4);
                }
            }
        }
    }

    // convert centerline poses
    if(_configuration >= 3) {
        std::vector<Eigen::Vector3d> convertedCenterline = centerlineConversion();
        // Draw centerline
        for (size_t i = 1; i < convertedCenterline.size(); ++i) {
            if(convertedCenterline[i](2) == LEFT_CAMERA) {
                cv::circle(leftImage->image, cv::Point(convertedCenterline[i](0), convertedCenterline[i](1)), 10, CV_RGB(0,255,0), cv::FILLED);
                if (i == convertedCenterline.size()-1) break;
                cv::line(leftImage->image, cv::Point(convertedCenterline[i](0), convertedCenterline[i](1)), cv::Point(convertedCenterline[i+1](0), convertedCenterline[i+1](1)), CV_RGB(0,0,255), 3);
            } else {
                cv::circle(rightImage->image, cv::Point(convertedCenterline[i](0), convertedCenterline[i](1)), 10, CV_RGB(0,255,0), cv::FILLED);
                if (i == convertedCenterline.size()-1) break;
                cv::line(rightImage->image, cv::Point(convertedCenterline[i](0), convertedCenterline[i](1)), cv::Point(convertedCenterline[i+1](0), convertedCenterline[i+1](1)), CV_RGB(0,0,255), 3);
            }
        }
    }

    // convert pointToFollow
    if(_configuration >= 4) {
        Eigen::Vector4d point(_pointToFollow.x, _pointToFollow.y, _pointToFollow.z, 0);
        std::vector<Eigen::Vector4d> points;
        points.push_back(point);
        std::vector<Eigen::Vector4d> convertedPointToFollow = pointsConversion(points);
        // Draw pointToFollow
        if(convertedPointToFollow[0](2) == LEFT_CAMERA) {
            cv::circle(leftImage->image, cv::Point(convertedPointToFollow[0](0), convertedPointToFollow[0](1)), 10, CV_RGB(0,0,255), cv::FILLED);
        } else {
            cv::circle(rightImage->image, cv::Point(convertedPointToFollow[0](0), convertedPointToFollow[0](1)), 10, CV_RGB(0,0,255), cv::FILLED);
        }
    }

    // convert cv image to image msg
    if(_numberCameras == 1){
        leftImage->header.frame_id = "visualization";
        leftImage->encoding = sensor_msgs::image_encodings::BGR8;
        _visualization = *leftImage->toImageMsg();
    } else {
        leftImage->header.frame_id = "visualization";
        leftImage->encoding = sensor_msgs::image_encodings::BGR8;
        rightImage->header.frame_id = "visualization";
        rightImage->encoding = sensor_msgs::image_encodings::BGR8;
        _leftCameraVisualization = *leftImage->toImageMsg();
        _rightCameraVisualization = *rightImage->toImageMsg();

        cv_bridge::CvImage cvJointImage;
        cv::Mat jointImage;
        cv::resize(leftImage->image, leftImage->image, cv::Size(2048,1536));
        cv::hconcat(leftImage->image, rightImage->image, jointImage);
        cvJointImage = cv_bridge::CvImage(leftImage->header, sensor_msgs::image_encodings::BGR8, jointImage);
        cvJointImage.toImageMsg(_visualization);
    }
}

/**
*	Name: getBoundingBoxColor.
*	Description:
*	Inputs: none
*	Output: void
*/
cv::Scalar Pierre::getBoundingBoxColor(int coneColor) {
    if (coneColor == BLUE_CONE) {
        return cv::Scalar(255,0,0);
    } else if (coneColor == YELLOW_CONE) {
        return cv::Scalar(0,255,255);
    } else if (coneColor == ORANGE_CONE) {
        return cv::Scalar(0,150,255);
    } else if (coneColor == BIG_ORANGE_CONE) {
        return cv::Scalar(0,0,255);
    } else {
        return cv::Scalar(150,150,150);
    }
}


/**
*	Name: extractPointsFromCloud.
*	Description: extract points from pointcloud
*	Inputs: none
*	Output: void
*/
std::vector<Eigen::Vector4d> Pierre::extractPointsFromCloud() {
    
    pcl::fromROSMsg(_lidarClusteredPoints, *_pclPointCloud);
    std::vector<Eigen::Vector4d> clusterPoints;
    for (int i = 0; i < _pclPointCloud->size(); i++){
        Eigen::Vector4d v(_pclPointCloud->points[i].x, _pclPointCloud->points[i].y, _pclPointCloud->points[i].z, 0);
        clusterPoints.push_back(v);
    }
    return clusterPoints;
}

void Pierre::transformConeDetections() {
	geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform("os_sensor",  _coneDetections.header.frame_id , ros::Time(0));

    for(common_msgs::Cone &cone : _coneDetections.cone_detections){
        cone.position.x += transform.transform.translation.x;
    }
}

/********  End of Pierre Processing  ********/