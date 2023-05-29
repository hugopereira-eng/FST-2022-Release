#include <graphslam/graphslam_pipeline.hpp>

using namespace Eigen;

/*****************************/
/********Visualization********/
/*****************************/

void GraphSlam::visualizeLandmarks() {
    
    _coneMarkers.markers.clear();
    _coneInfoMarkers.markers.clear();
    visualization_msgs::Marker marker;
    visualization_msgs::Marker textMarker;
    marker.header.frame_id = "map";
    marker.action = visualization_msgs::Marker::DELETEALL;
    _coneMarkers.markers.push_back(marker);
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(0);
    marker.header.stamp = _coneDetections.header.stamp;
    textMarker = marker;
    textMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    
    for (int i = 0; i < _landmarks.size(); ++i) {
        if(_landmarks[i].numObserved <= 0) continue;
        marker.pose.position.x = _landmarks[i].mean.x();
        marker.pose.position.y = _landmarks[i].mean.y();
        marker.pose.orientation = tf::createQuaternionMsgFromYaw(0);
        marker.pose.position.z = 0;
        marker.id = i + 1;

        textMarker.pose.position = marker.pose.position;
        textMarker.pose.position.z = 0.5;
        textMarker.id = i + 1;
        textMarker.scale.z = 0.2;
		textMarker.color.a = 1;
		textMarker.color.r = 0;
		textMarker.color.g = 0;
		textMarker.color.b = 0;
        textMarker.text = "ID:"+std::to_string(_landmarks[i].index) + " NumObserved:"+std::to_string(_landmarks[i].numObserved);


        if(_landmarks[i].color == BLUE_CONE) {
            marker.color.r = 0;
            marker.color.g = 0;
            marker.color.b = 1;
        } else if(_landmarks[i].color == YELLOW_CONE) {
            marker.color.r = 1;
            marker.color.g = 1;
            marker.color.b = 0;
        } else if (_landmarks[i].color == ORANGE_CONE || _landmarks[i].color == BIG_ORANGE_CONE) {
            marker.color.r = 1;
            marker.color.g = 0.65;
            marker.color.b = 0;
        } else {

            marker.color.r = 0.5;
            marker.color.g = 0.5;
            marker.color.b = 0.5;
        }
        _coneMarkers.markers.push_back(marker);
        _coneInfoMarkers.markers.push_back(textMarker);
    }  
}

void GraphSlam::currentDetectionsVisualization() {
    
    _currentConeMarkers.markers.clear();
    _currentConeDetections.cone_detections.clear();

    float nearestLandmarkLeftDistance = std::numeric_limits<float>::max();
    float nearestLandmarkRightDistance = std::numeric_limits<float>::max();
    Landmark *nearestLandmarkLeft = nullptr;
    Landmark *nearestLandmarkRight = nullptr;
    int counter = 1;
    visualization_msgs::Marker marker;
    marker.action = visualization_msgs::Marker::DELETEALL;
    _currentConeMarkers.markers.push_back(marker);
    for(int i = 0; i < _landmarks.size(); ++i) {
        Landmark landmark = _landmarks[i];
        if(landmark.numObserved <= 0) 
            continue; // dont publish cones that were only observed once
        
        Observation obs = landmark2Observation(_landmarks[i], _pose);
        float dx = landmark.mean.x() - _pose.pos.x();
        float dy = landmark.mean.y() - _pose.pos.y();
        // if(dy > -5 && dy < 5 && dx <= 10) 
        if(obs.measure.theta > -M_PI_2 && obs.measure.theta < M_PI_2 && obs.measure.d <= _associationRange) {
            //Cone Detections
            common_msgs::Cone cone;
            cone = observation2Cone(obs);
            _currentConeDetections.cone_detections.push_back(cone);
            
            //Visualization
            marker.header.stamp = _coneDetections.header.stamp;
            marker.header.frame_id = _transform.header.frame_id;
            marker.ns = "Current Cone Detections";
            marker.id = counter;
            marker.scale.x = 0.5;
            marker.scale.y = 0.5;
            marker.scale.z = 0.5;   
            marker.color.a = 1;	
            marker.type = 3;
            marker.pose.orientation = tf::createQuaternionMsgFromYaw(0);
            //marker.lifetime = ros::Duration(0.05);
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = cone.position.x;
            marker.pose.position.y = cone.position.y;
            marker.pose.position.z = 0;

            if (_landmarks[i].color == YELLOW_CONE) {
                marker.color.r = 1;
                marker.color.g = 1;
                marker.color.b = 0;
            } else if(_landmarks[i].color == BLUE_CONE) {
                marker.color.r = 0;
                marker.color.g = 0;
                marker.color.b = 1;
            } else if(_landmarks[i].color == ORANGE_CONE || _landmarks[i].color == BIG_ORANGE_CONE) {
                marker.color.r = 1;
                marker.color.g = 0.65;
                marker.color.b = 0;
            } else {
                marker.color.r = 0.5;
                marker.color.g = 0.5;
                marker.color.b = 0.5;
            }
            _currentConeMarkers.markers.push_back(marker);
            counter++;

        } else if (obs.measure.theta > M_PI_2 && obs.measure.theta < nearestLandmarkLeftDistance) {
            nearestLandmarkLeftDistance = obs.measure.d;
            nearestLandmarkLeft = &landmark;
        } else if (obs.measure.theta < -M_PI_2 && obs.measure.theta < nearestLandmarkRightDistance) {
            nearestLandmarkRightDistance = obs.measure.d;
            nearestLandmarkRight = &landmark;
        }
    }

    // if (nearestLandmarkLeft) {
    //     _currentConeDetections.cone_detections.push_back(observation2Cone(landmark2Observation(nearestLandmarkLeft, &pose), nearestLandmarkLeft));
    // }
    // if (nearestLandmarkRight) {
    //     _currentConeDetections.cone_detections.push_back(observation2Cone(landmark2Observation(nearestLandmarkRight, &pose), nearestLandmarkRight));
    // }

    _currentConeDetections.header.frame_id = _coneDetections.header.frame_id;
    _currentConeDetections.header.stamp = _coneDetections.header.stamp;
}

/* void GraphSlam::visualizeCurrentObservations(std::vector<Observation> observations, Pose pose) {
	_coneObservationMarkers.markers.clear();

    visualization_msgs::Marker marker;
    marker.action = visualization_msgs::Marker::DELETEALL;
    _coneObservationMarkers.markers.push_back(marker);

	marker.header.stamp = _coneDetections.header.stamp;
	marker.header.frame_id = _coneDetections.header.frame_id;
	marker.ns = "Current Observations";
	marker.scale.x = 0.5;
	marker.scale.y = 0.5;
	marker.scale.z = 0.5;   
	marker.color.a = 1;	
	marker.type = 2;
	marker.pose.orientation = tf::createQuaternionMsgFromYaw(0);
	marker.action = visualization_msgs::Marker::ADD;

	for (size_t i = 0; i < observations.size(); ++i) {
        
        common_msgs::Cone conePose = observation2Cone(observations[i], &_landmarks[0]);

        marker.pose.position.x = conePose.position.x;
        marker.pose.position.y = conePose.position.y;
        marker.pose.position.z = 0;
        marker.id = i + 1;
	    if (observations[i].color == YELLOW_CONE) {
	    	marker.color.r = 1;
			marker.color.g = 1;
			marker.color.b = 0;
	    } else if(observations[i].color == BLUE_CONE) {
	    	marker.color.r = 0;
			marker.color.g = 0;
			marker.color.b = 1;
	    } else if(observations[i].color == ORANGE_CONE || observations[i].color == BIG_ORANGE_CONE) {
			marker.color.r = 1;
			marker.color.g = 0.65;
			marker.color.b = 0;
		} else {
			marker.color.r = 0.5;
			marker.color.g = 0.5;
			marker.color.b = 0.5;
		}
        _coneObservationMarkers.markers.push_back(marker);
	}
} */

/* void GraphSlam::visualizeDataAssociation(std::vector<bool> observedLandmarks) {
	
    _dataAssociationMarkers.markers.clear();
    visualization_msgs::Marker marker;

    marker.header.stamp = _coneDetections.header.stamp;
	marker.header.frame_id = "map";
    marker.action = visualization_msgs::Marker::DELETEALL;
    _dataAssociationMarkers.markers.push_back(marker);
	marker.ns = "Cone Info";
    marker.action = visualization_msgs::Marker::ADD;
	marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	marker.scale.z = 0.2;   
	marker.color.a = 1;	
    marker.color.r = 0;
	marker.color.g = 0;
	marker.color.b = 0;
	marker.pose.orientation = tf::createQuaternionMsgFromYaw(0);
	
    for (int i = 0; i < observedLandmarks.size(); i++) {
		if (observedLandmarks[i]) {
            marker.ns = "Cone Info";
            marker.id = i + 1;
            marker.pose.position.x = _landmarks[i].mean.x();
            marker.pose.position.y = _landmarks[i].mean.y();
            marker.pose.position.z = -0.5;
            marker.text = "Landmark ID:" + std::to_string(i) + "\n Tracking ID:" + std::to_string(_landmarks[i].tracking)
                                + "\n Observed:" + std::to_string(_landmarks[i].numObserved);
            _dataAssociationMarkers.markers.push_back(marker);
        }
	}
} */

void GraphSlam::validateDataAssociation() {
    _daMarkers.markers.clear();
    _edges.points.clear();
    int index = 0;
    visualization_msgs::Marker marker;
    marker.header.stamp = _coneDetections.header.stamp;
	marker.header.frame_id = "map";
    marker.action = visualization_msgs::Marker::DELETEALL;
    _daMarkers.markers.push_back(marker);

    marker.header.stamp = _coneDetections.header.stamp;
	marker.header.frame_id = "map";
	marker.ns = "Current";
	marker.scale.x = 0.2;
	marker.scale.y = 0.2;
	marker.scale.z = 0.2;   
	marker.color.a = 1;	
	marker.type = 2;
	marker.pose.orientation = tf::createQuaternionMsgFromYaw(0);
	marker.action = visualization_msgs::Marker::ADD;
    
    geometry_msgs::Point point1;
    geometry_msgs::Point point2;
    _edges.header.frame_id = "map";
    _edges.header.stamp = ros::Time::now();
    _edges.lifetime = ros::Duration(0);
    
    _edges.ns = "da edges";
    _edges.action = visualization_msgs::Marker::ADD;
    _edges.type = visualization_msgs::Marker::LINE_LIST;
    _edges.scale.x = 0.03;
    _edges.color.a = 1.0;
    _edges.color.r = 0.0;
    _edges.color.g = 0.0;
    _edges.color.b = 0.0;

    for (Landmark &landmark: _landmarks) {
        marker.pose.position.x = landmark.mean.x();
        marker.pose.position.y = landmark.mean.y();
        marker.pose.position.z = 0;
        marker.id = index + 1;
        index ++;
        marker.color.r = 1;
		marker.color.g = 0;
		marker.color.b = 0;
        _daMarkers.markers.push_back(marker);
        point1.x = landmark.mean.x();
        point1.y = landmark.mean.y();
        point1.z = 0;
        for (Vector2d &observation: landmark.observations) {
            marker.pose.position.x = observation.x();
            marker.pose.position.y = observation.y();
            marker.pose.position.z = 0;
            marker.id = index + 1;
            index++;
            marker.color.r = 0;
		    marker.color.g = 1;
		    marker.color.b = 0;
            _daMarkers.markers.push_back(marker);
            point2.x = observation.x();
            point2.y = observation.y();
            point2.z = 0;
            _edges.points.push_back(point1);
            _edges.points.push_back(point2);

        }
    }
        
}