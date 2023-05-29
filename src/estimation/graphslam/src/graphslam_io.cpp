#include <graphslam/graphslam_pipeline.hpp>

/*****************************/
/*************IO**************/
/*****************************/

// loads a map from a previous run (special g2o file)
void GraphSlam::loadMap(ros::NodeHandle &nodeHandle) {
    _landmarks.clear();
    _centerLine.poses.clear();

    XmlRpc::XmlRpcValue coneArray;
    std::vector<Eigen::Vector2d> yellowCones, blueCones, leftOrangeCones, rightOrangeCones, bigOrangeCones, centerline;
    
    //Load Yellow Cones
    nodeHandle.getParam("/map/cones/yellow_cones", coneArray);
    convertXMLRPCVector(yellowCones, coneArray);
    convert2Landmark(yellowCones, _landmarks, YELLOW_CONE, LmSide::Left, _observationNoise, _observationIncrement);
    
    //Load Blue Cones
    nodeHandle.getParam("/map/cones/blue_cones", coneArray);
    convertXMLRPCVector(blueCones, coneArray);
    convert2Landmark(blueCones, _landmarks, BLUE_CONE, LmSide::Right, _observationNoise, _observationIncrement);
    
    // //Load Orange Cones
    nodeHandle.getParam("/map/cones/orange_cones", coneArray);
    convertXMLRPCVector(leftOrangeCones, coneArray);
    convert2Landmark(leftOrangeCones, _landmarks, ORANGE_CONE, LmSide::Unknown, _observationNoise, _observationIncrement);
    
    //Load Big Orange Cones
    nodeHandle.getParam("/map/cones/big_orange_cones", coneArray);
    convertXMLRPCVector(bigOrangeCones, coneArray);
    convert2Landmark(bigOrangeCones, _landmarks, BIG_ORANGE_CONE, LmSide::Unknown, _observationNoise, _observationIncrement);
    
    //Load Centerline
    nodeHandle.getParam("/map/centerline", coneArray);
    convertXMLRPCVector(centerline, coneArray);
    convert2CenterPoint(centerline, _centerLine);
    _centerLine.header.frame_id = "map";
    _centerLine.header.stamp = ros::Time::now();
      
    // create graph 
    for (Landmark &lm : _landmarks) {
        _graphTypes.addLandmarkVertex(lm, _graphIndex);
        lm.index = _graphIndex++;
    }
    
    _graphTypes.fixGraph();
}

// saves the current graph to a map. can be optimized or non optimized.
// special g2o file that can later be opened in the g2o GUI app
void GraphSlam::saveMap() {

    std::ofstream mapFile;
    std::string fileName = _mapPath + "/trackdrive.yaml";
    mapFile.open(fileName);

    std::vector<Eigen::Vector2d> yellowCones, blueCones, orangeCones, bigOrangeCones, centerLine;

    for (Landmark &landmark: _landmarks) {
        if (landmark.color == YELLOW_CONE) {
            yellowCones.push_back(Eigen::Vector2d(landmark.mean.x(), landmark.mean.y()));
        } else if (landmark.color == BLUE_CONE) {
            blueCones.push_back(Eigen::Vector2d(landmark.mean.x(), landmark.mean.y()));
        } else if (landmark.color == ORANGE_CONE) {
            orangeCones.push_back(Eigen::Vector2d(landmark.mean.x(), landmark.mean.y()));
        } else if (landmark.color == BIG_ORANGE_CONE) {
            bigOrangeCones.push_back(Eigen::Vector2d(landmark.mean.x(), landmark.mean.y()));
        }
    }

    for (auto& pos: _centerLine.poses){
        centerLine.push_back(Eigen::Vector2d(pos.pose.position.x, pos.pose.position.y));
    }

    auto addToFile = [&](std::vector<Eigen::Vector2d>& vec, std::string name){
        mapFile << "        " << name << ": [";
        //mapFile << "        [";

        for (int i = 0; i < vec.size(); i++) {
            if (i == 0) {
                mapFile << "[";
            } else {
                mapFile << "        [";
            }
            mapFile << std::fixed << std::setprecision(3) << vec[i].x();
            mapFile << ", ";
            mapFile << std::fixed << std::setprecision(3) << vec[i].y();
            if (i == vec.size()-1) {
                mapFile << "]";
            } else {
                mapFile << "],\n"; 
            }     
        }
        mapFile << "]\n\n";
    };

    mapFile << "map: \n";
    mapFile << "    cones: \n";
    
    //Yellow Cones
    addToFile(yellowCones, "yellow_cones");
    
    //Blue cones
    addToFile(blueCones, "blue_cones");
    
    //Orange cones
    addToFile(orangeCones, "orange_cones");

    //Big Orange cones
    addToFile(bigOrangeCones, "big_orange_cones");

    //Centerline
    addToFile(centerLine, "centerline");

    mapFile.close();
}