#include "graphslam/graphslam_icp.hpp"


common_msgs::ConeDetections const & ICP_odom::getCurrentConeDetections() const {return _currentConeDetections;}
sensor_msgs::PointCloud2 const & ICP_odom::getAlignedPoincloud() const { return _alignedPointCloud;}
sensor_msgs::PointCloud2 const & ICP_odom::getSourcePointcloud() const { return _sourcePointCloud;}
sensor_msgs::PointCloud2 const & ICP_odom::getTargetPointcloud() const { return _targetPointCloud;}
nav_msgs::Odometry const & ICP_odom::getIcpOdometry() const {return _icpOdometry;}

ICP_odom::ICP_odom(){}

void ICP_odom::run(const common_msgs::ConeDetections &coneDetections) {
    icpWihtoutOutliers(coneDetections);
}

void ICP_odom::icpWihtoutOutliers(const common_msgs::ConeDetections &coneDetections) {	


    if (coneDetections.cone_detections.empty()) return;

    int k = 0;      // iterator
    float p = 0.9999; // probability confidence
    float e = 0.5; // outlier_ratio
    int s = 10;     // number of points to sample the point cloud
    
    float tx,ty,tz = 0;

    //int n_ransac_iterations = ceil( log(1 - p) / log(1 -     ROS_WARN("ICP");pow(1-e, s) ) );
    int n_ransac_iterations = 1000;
    int random_index;
    PclPoint random_source_point;
    PclPoint random_target_point;
    PclPoint transformed_point;
    float d, dx, dy, theta, r;
    float min_dx, min_dy, min_theta;
    float avg_error;
    float min_avg_error = 1000; // infinity
    int NN_index;

    // ICP initial guess
    Eigen::Matrix4f initial_guess;

    pcl::IterativeClosestPoint<PclPoint, PclPoint> icp;
    PclPointCloud::Ptr sourceCloud (new PclPointCloud);
    PclPointCloud::Ptr filtered_sourceCloud (new PclPointCloud);
    PclPointCloud::Ptr filtered_targetCloud (new PclPointCloud);
    PclPointCloud::Ptr outputCloud (new PclPointCloud);
    PclPointCloud::Ptr targetCloud (new PclPointCloud);
    PclPoint point;
    Eigen::Matrix4f icpTransformMatrix, currentTransformMatrix;

    //Observation to pointcloud
    for (int i = 0; i < coneDetections.cone_detections.size(); i++) {
        point.x = coneDetections.cone_detections[i].position.x;
        point.y = coneDetections.cone_detections[i].position.y;
        point.z = 0;
        targetCloud->push_back(point);
    }
    _targetPointCloud = convertPcl2Ros(targetCloud);
    _targetPointCloud.header = _detectionsHeader;

    if (_icpInit == true) {
        for (int i = 0; i < _previousConeDetections.cone_detections.size(); i++ ) {
            point.x = _previousConeDetections.cone_detections[i].position.x;
            point.y = _previousConeDetections.cone_detections[i].position.y;
            point.z = 0;
            sourceCloud->push_back(point);
        }
      
    } else {
        sourceCloud = targetCloud;
    }

    float x1,x2,y1,y2,x0,y0;
    bool repeated = false;
    std::vector<int> tmp_inliers_target_indexes,final_inliers_target_indexes;     
    float cnt_inliers_target = 0;
    float min_inliers_target = 1000;
    
    std::vector<int> tmp_inliers_source_indexes,final_inliers_source_indexes;     
    float cnt_inliers_source = 0;
    float min_inliers_source = 1000;


    // Ransac outlier handling
    if (_icpInit == true) {

        
        for(k = 0; k < n_ransac_iterations; k++ )
        {
            cnt_inliers_target = 0;
            tmp_inliers_target_indexes.clear();

            cnt_inliers_source = 0;
            tmp_inliers_source_indexes.clear();

            // Sample target point
            srand((unsigned int)time(NULL));
            random_index = rand() % ( coneDetections.cone_detections.size() - 1 );
            random_target_point.x = coneDetections.cone_detections[random_index].position.x;
            random_target_point.y = coneDetections.cone_detections[random_index].position.y;
            random_target_point.z = 0;
        
            // Sample source point
            srand((unsigned int)time(NULL));
            random_index = rand() % ( _previousConeDetections.cone_detections.size() - 1 );
            random_source_point.x = _previousConeDetections.cone_detections[random_index].position.x;
            random_source_point.y = _previousConeDetections.cone_detections[random_index].position.y;
            random_source_point.z = 0;
            
            // Target
            cnt_inliers_target = 0;
            // apply transformation from each point on the source point cloud
            for (int i = 0; i < coneDetections.cone_detections.size(); i++ ) {
                x1 = random_target_point.x;
                y1 = random_target_point.y;
                x2 = random_source_point.x;
                y2 = random_source_point.y;
                x0 = coneDetections.cone_detections[i].position.x;
                y0 = coneDetections.cone_detections[i].position.y;
                
                if ( distance_to_line(x1,y1,x2,y2,x0,y0) < 20.0 )
                {
                    cnt_inliers_target++;
                    tmp_inliers_target_indexes.push_back(i);
                }

            }

            if(cnt_inliers_target < min_inliers_target) {
                min_inliers_target = cnt_inliers_target;
                final_inliers_target_indexes = tmp_inliers_target_indexes;
            }

            // Source
            cnt_inliers_source = 0;
            // apply transformation from each point on the source point cloud
            for (int i = 0; i < _previousConeDetections.cone_detections.size(); i++ ) {
                x1 = random_source_point.x;
                y1 = random_source_point.y;
                x2 = random_target_point.x;
                y2 = random_target_point.y;
                x0 = _previousConeDetections.cone_detections[i].position.x;
                y0 = _previousConeDetections.cone_detections[i].position.y;
                
                if ( distance_to_line(x1,y1,x2,y2,x0,y0) < 20.0 )
                {
                    cnt_inliers_source++;
                    tmp_inliers_source_indexes.push_back(i);
                }

            }

            if(cnt_inliers_source < min_inliers_source) {
                min_inliers_source = cnt_inliers_source;
                final_inliers_source_indexes = tmp_inliers_source_indexes;
            }
                

        }       

        //Observation to pointcloud
        for(int i = 0; i < final_inliers_target_indexes.size(); i++) {

            point.x = coneDetections.cone_detections[ final_inliers_target_indexes[i] ].position.x;
            point.y = coneDetections.cone_detections[ final_inliers_target_indexes[i] ].position.y;
            point.z = 0;
            filtered_targetCloud->push_back(point);
        }

        //Observation to pointcloud
        for(int i = 0; i < final_inliers_source_indexes.size(); i++) {

            point.x = _previousConeDetections.cone_detections[ final_inliers_source_indexes[i] ].position.x;
            point.y = _previousConeDetections.cone_detections[ final_inliers_source_indexes[i] ].position.y;
            point.z = 0;
            filtered_sourceCloud->push_back(point);
        }

        _sourcePointCloud = convertPcl2Ros(filtered_sourceCloud);
        _sourcePointCloud.header = _detectionsHeader;

        _targetPointCloud = convertPcl2Ros(filtered_targetCloud);
        _targetPointCloud.header = _detectionsHeader;

        // Motion model initial guess
        tx = _pose.pos.x() - _prevPose.pos.x(); //_icpPose(0);
        ty = _pose.pos.y() - _prevPose.pos.y(); // _icpPose(1);
        tz = _pose.theta - _prevPose.theta; //_icpPose(2);

        _prevPose = _pose;

        // Initial guess
        
        // Yaw to rotation matrix
        Eigen::Affine3f transformatoin;

        initial_guess(0,3) = tx; //min_dx; // tx
        initial_guess(1,3) = ty; //min_dy; // ty
        transformatoin = pcl::getTransformation(tx,ty,0,0,0,tz);
        initial_guess(2,3) = 0;           // tz
        initial_guess(3,3) = 1;           // normalization

        // Rotation (rest is all zeros)
        initial_guess(0,0) = transformatoin (0,0); initial_guess(0,1) = transformatoin (0,1); initial_guess(0,2) = transformatoin (0,2);
        initial_guess(1,0) = transformatoin (1,0); initial_guess(1,1) = transformatoin (1,1); initial_guess(1,2) = transformatoin (1,2);
        initial_guess(2,0) = transformatoin (2,0); initial_guess(2,1) = transformatoin (2,1); initial_guess(2,2) = transformatoin (2,2);
    }

	//icp.setInputSource(sourceCloud);
    icp.setInputSource(filtered_sourceCloud);
	icp.setInputTarget(filtered_targetCloud);
    icp.setMaxCorrespondenceDistance(1.0);
    icp.setMaximumIterations(100);
    if (_icpInit == true)
        icp.align(*outputCloud, initial_guess);
        //icp.align(*outputCloud);
    else
        icp.align(*outputCloud);

	if (icp.hasConverged()) {
		std::cout << "ICP converged." << std::endl;
		std::cout << "Transformation matrix:" << std::endl;
		
        icpTransformMatrix = icp.getFinalTransformation();
        std::cout << icpTransformMatrix << std::endl;
    
        _icpPose(0) -= cosf(_icpPose(2)) * icpTransformMatrix(0,3) - sinf(_icpPose(2)) * icpTransformMatrix(1,3);
        _icpPose(1) -= sinf(_icpPose(2)) * icpTransformMatrix(0,3) + cosf(_icpPose(2)) * icpTransformMatrix(1,3);
        //_icpPose(2) += atan2(icpTransformMatrix(1,0), icpTransformMatrix(0,0));
        _icpPose(2) = _pose.theta;
        computeIcpOdometry();

	} else { std::cout << "ICP did not converge." << std::endl; }

    _alignedPointCloud = convertPcl2Ros(outputCloud);
    _alignedPointCloud.header = _detectionsHeader;
    _previousConeDetections = coneDetections; 
    _icpInit = true; 
}

float ICP_odom::distance_to_line(float x1,float y1, float x2, float y2, float x0, float y0)
{
    float dx = x2-x1; float dy = y2-y1;
    float d = std::hypot(dx, dy);
    float deviation = abs(dx*(y1-y0) - (x1-x0)*dy);
    return deviation/d;
}

void ICP_odom::computeIcpOdometry() {
    _icpOdometry.header.frame_id = "map";
    _icpOdometry.header.stamp = _detectionsHeader.stamp;
    _icpOdometry.pose.pose.position.x = _icpPose(0);
    _icpOdometry.pose.pose.position.y = _icpPose(1);
    _icpOdometry.pose.pose.orientation = tf::createQuaternionMsgFromYaw(_icpPose(2));
}

sensor_msgs::PointCloud2 ICP_odom::convertPcl2Ros(PclPointCloud::Ptr pclPointCloud) {
    sensor_msgs::PointCloud2 pointCloud;
    pcl::toROSMsg(*pclPointCloud, pointCloud);
    return pointCloud;
}
