#include <ros/ros.h>
#include "path_planner/path_planner_path_tree.hpp"
#include <unistd.h>
#include <eigen3/Eigen/Eigen>


PathPlannerTree::PathPlannerTree() {
    ROS_INFO("Constructing PathPlannerTree");
	if (!ros::param::get("cost_function_Kwidth", _Kwidth)) {
		ROS_WARN_STREAM("Could not load cost_function/Kwidth. Default value is 5");
		_Kwidth = 5;
	}
	if (!ros::param::get("cost_function_Kdistance", _Kdistance))	{
		ROS_WARN_STREAM("Could not load cost_function/Kdistance. Default value is 1");
		_Kdistance = 1;
	}
	if (!ros::param::get("cost_function_Kangle", _Kangle)) {
		ROS_WARN_STREAM("Could not load cost_function/Kangle. Default value is 5");
		_Kangle = 5;
	}
	if (!ros::param::get("cost_function_Kdistance_total", _Kdistance_total)) {
		ROS_WARN_STREAM("Could not load cost_function/Kdistance_total. Default value is 1");
		_Kdistance_total = 1;
	}
	if (!ros::param::get("cost_function_coneColor", _coneColor)) {
		ROS_WARN_STREAM("Could not load cost_function/coneColor. Default value is 1000");
		_coneColor = 1000;
	}
}

PathPlannerTree::~PathPlannerTree() {
    cleanDataStructures();
}

void PathPlannerTree::cleanDataStructures() {
    _leafnode.clear();
    _generatedList.clear();
    _bestTrajectory.poses.clear();
    _delaunayMarkerLines.points.clear();
    _midpointConnectionsLines.points.clear();
    _bestTrajectoryMarkers.markers.clear();
}

void PathPlannerTree::deleteAllTree() {
    deleteTree(_root);
}

void PathPlannerTree::deleteTree(node *root) {
    //do nothing if passed a non-existent node
    if(root == NULL)
        return;
    else if (root == _root) {
        for (auto &parentChild: _root->parentChilds) {
            deleteTree(parentChild);    
        }
    }
    //now onto the recursion
    deleteTree(root->leftChild);
    deleteTree(root->rightChild);
    delete root;
}

//getters
nav_msgs::Path const & PathPlannerTree::getBestPath() const { return _bestTrajectory; }

visualization_msgs::MarkerArray const & PathPlannerTree::getBestTrajectoryMarkers() const { return _bestTrajectoryMarkers; }
visualization_msgs::Marker const & PathPlannerTree::getDelaunayMarkerLines() const { return _delaunayMarkerLines; }
visualization_msgs::Marker const & PathPlannerTree::getMidpointConnectionsLines() const { return _midpointConnectionsLines; }
common_msgs::Cost const & PathPlannerTree::getBestCost() const { return _bestCost; }
common_msgs::Cost const & PathPlannerTree::getSecondBestCost() const { return _secondBestCost; }
common_msgs::Cost const & PathPlannerTree::getThirdBestCost() const { return _thirdBestCost; }

//setters
void PathPlannerTree::setKwidth(double Kwidth){_Kwidth = Kwidth;}
void PathPlannerTree::setKdistance(double Kdistance){_Kdistance = Kdistance;}
void PathPlannerTree::setKangle(double Kangle){_Kangle = Kangle;}
void PathPlannerTree::setKdistance_total(double Kdistance_total){_Kdistance_total = Kdistance_total;}
void PathPlannerTree::setconeColor(double coneColor){_coneColor = coneColor;}
void PathPlannerTree::setconeNoColor(double coneNoColor){_coneNoColor = coneNoColor;}
void PathPlannerTree::setwrongSide(double wrongSide){_wrongSide = wrongSide;}

void PathPlannerTree::setWidthAdmMax(double widthAdmMax){_widthAdmMax = widthAdmMax;}
void PathPlannerTree::setWidthAdmMin(double widthAdmMin){_widthAdmMin = widthAdmMin;}
void PathPlannerTree::setDistanceAdmMax(double distanceAdmMax){_distanceAdmMax = distanceAdmMax;}
void PathPlannerTree::setDistanceAdmMin(double distanceAdmMin){_distanceAdmMin = distanceAdmMin;}
void PathPlannerTree::setAngleAdm(double angleAdm){_angleAdm = angleAdm;}
void PathPlannerTree::setDistanceTotalAdmMax(double distanceTotalAdmMax){_distanceTotalAdmMax = distanceTotalAdmMax;}
void PathPlannerTree::setDistanceTotalAdmMin(double distanceTotalAdmMin){_distanceTotalAdmMin = distanceTotalAdmMin;}


void PathPlannerTree::generateTree(Delaunay &dt) {

    cleanDataStructures();
    _dt = dt;
    
    struct node *originNode = new node;
    originNode->data.midpoint.pose.position.x = 0;
    originNode->data.midpoint.pose.position.y = 0;
    originNode->data.cost = calculateCost(originNode);
    originNode->data.pathSteps = 0;
    _root = originNode;
    generateFirstElement();

    while(!_queue.empty()) {
        struct node *nodeToExpand = _queue.front();
        _queue.pop();
        generateChildNodes(nodeToExpand);
    }
}


void PathPlannerTree::generateFirstElement() {


    // find nearest vertex
    Delaunay::Vertex_handle nearestVertexHandle;
    nearestVertexHandle = _dt.nearest_vertex(Point(0, 0));
    //std::cout << "nearest point is (" << nearestVertexHandle->point().x() << ", " << nearestVertexHandle->point().y() << ")" << std::endl;
    
    //iteration through all midpoints that can be generated 
    Delaunay::Edge_circulator edgeCirculator = _dt.incident_edges(nearestVertexHandle), done(edgeCirculator);

    do {
        struct node *firstMidPointNode = new node;
        if(!_dt.is_infinite(edgeCirculator)) {

            /* we need to check if the edge is part of a infinite face because we want the first face to be he ininite one so that
             * the first search is done in the first finite face 
             */
            if(!_dt.is_infinite(edgeCirculator->first)) {
                firstMidPointNode->data.faceHandle = _dt.mirror_edge(*edgeCirculator).first;
                firstMidPointNode->data.oppositeVertexIndex = _dt.mirror_edge(*edgeCirculator).second;
            }
            else {
                firstMidPointNode->data.faceHandle = edgeCirculator->first;
                firstMidPointNode->data.oppositeVertexIndex = edgeCirculator->second;
            }

            firstMidPointNode->parent = _root;
            firstMidPointNode->data.midpoint = calculateMidpoint(edgeCirculator->first->vertex(edgeCirculator->first->cw(edgeCirculator->second)), edgeCirculator->first->vertex(edgeCirculator->first->ccw(edgeCirculator->second)));    
            firstMidPointNode->data.pathSteps = 1;
            firstMidPointNode->data.cost = calculateCost(firstMidPointNode);
            _root->parentChilds.push_back(firstMidPointNode);
            _generatedList.push_back(firstMidPointNode);
            _queue.push(firstMidPointNode);
            
            if(!_dt.is_infinite(firstMidPointNode->data.faceHandle)) {
                struct node *firstMidPointMirrored = new node;
                firstMidPointMirrored = new node;
                firstMidPointMirrored->data.faceHandle = edgeCirculator->first;
                firstMidPointMirrored->data.oppositeVertexIndex = edgeCirculator->second;
                firstMidPointMirrored->parent = _root;
                firstMidPointMirrored->data.midpoint = firstMidPointNode->data.midpoint;
                firstMidPointMirrored->data.cost = firstMidPointNode->data.cost;
                firstMidPointMirrored->data.pathSteps = 1;
                _root->parentChilds.push_back(firstMidPointMirrored);
                _generatedList.push_back(firstMidPointMirrored);
                _queue.push(firstMidPointMirrored);
                //std::cout << "added first mirrored midpointt at (" << firstMidPointMirrored->data.midpoint.pose.position.x << ", " << firstMidPointMirrored->data.midpoint.pose.position.y << ") with cost - " <<  firstMidPointMirrored->data.cost.costTotal << std::endl;
            }                        
        }
        else {
            delete firstMidPointNode;
        }   
    } while (++edgeCirculator != done);    
} 


bool PathPlannerTree::keepTriangle(Delaunay::Face_handle face){

    //get the three vertecies of a face
    Delaunay::Vertex_handle v1 = face->vertex(0);
    Delaunay::Vertex_handle v2 = face->vertex(1);
    Delaunay::Vertex_handle v3 = face->vertex(2);

    //std::cout << "---------- Analysing new trinagle ----------" << std::endl;
    //std::cout << v1->point() << " " << v2->point() << " " << v3->point() << std::endl;

    //use the 3 vertecies to compute the 3 edges of a face
    Eigen::Vector2f edge1(v1->point().x() - v2->point().x(), v1->point().y() - v2->point().y()); // 1 -> 2
    Eigen::Vector2f edge2(v2->point().x() - v3->point().x(), v2->point().y() - v3->point().y()); // 2 -> 3
    Eigen::Vector2f edge3(v3->point().x() - v1->point().x(), v3->point().y() - v1->point().y()); // 3 -> 1

    //check if at least 2 edges have a size greater or equal than 3, because we are not interested in small triangles (minimum track width)
    uint8_t edgeSize = 0;
    if (edge1.norm() >= 3.0) edgeSize++;
    if (edge2.norm() >= 3.0) edgeSize++;
    if (edge3.norm() >= 3.0) edgeSize++;
    if (edgeSize < 2) return false;

    //get the inner angle of each vertex from to the cosine law
    float angle1 = std::acos((edge1.dot(edge1) + edge2.dot(edge2) - edge3.dot(edge3))/(2*edge1.norm()*edge2.norm()));
    float angle2 = std::acos((edge2.dot(edge2) + edge3.dot(edge3) - edge1.dot(edge1))/(2*edge2.norm()*edge3.norm()));
    float angle3 = std::acos((edge3.dot(edge3) + edge1.dot(edge1) - edge2.dot(edge2))/(2*edge3.norm()*edge1.norm()));

    //exclude triangles with an angle too big
    if (angle1 >= M_PI*(5.0/6) || angle2 >= M_PI*(5.0/6) || angle3 >= M_PI*(5.0/6)) return false;

    return true;
}

geometry_msgs::PoseStamped PathPlannerTree::calculateMidpoint(Delaunay::Vertex_handle v1, Delaunay::Vertex_handle v2) {
    geometry_msgs::PoseStamped midpoint;
    midpoint.pose.position.x = (v1->point().x() + v2->point().x())/ 2.0;
	midpoint.pose.position.y = (v1->point().y() + v2->point().y())/ 2.0;
    return midpoint;
}

CostData PathPlannerTree::calculateCost(node *childNode) {
    
    costData cost;
    float distance = 0;
    float distance_child2GP = 0;
    float distance_parent2GP = 0;
    float width = 0;
    float angle = 0;
    float coneColor = 0;
    float coneNoColor = 0;
    float going_back = 0;
    node *grand_parent = childNode;

    float wrong_side = 0;
    float angle_with_vertex1 = 0;
    float angle_with_vertex2 = 0;
    double u1, u2, v1, v2, cone_coordinate11, cone_coordinate12, cone_coordinate21, cone_coordinate22, cone_coordinate1, cone_coordinate2;

    if (childNode->parent == NULL) {
        cost.costTotal = 0;
        
        childNode->data.cost.costWidth = 0;
        childNode->data.cost.costDistance = 0;
        childNode->data.cost.costAngle = 0;
        childNode->data.cost.costDistanceTotal = 0;
        childNode->data.cost.costColor = 0;
        childNode->data.totalDistance = 0;

    } else{

        distance = hypot((childNode->data.midpoint.pose.position.x - childNode->parent->data.midpoint.pose.position.x), (childNode->data.midpoint.pose.position.y - childNode->parent->data.midpoint.pose.position.y));

        childNode->data.totalDistance = childNode->parent->data.totalDistance + distance;
            

        // width of the track, measured from one vertice of the midpoint to the other:
        auto vertex1 = childNode->data.faceHandle->vertex(childNode->data.faceHandle->cw(childNode->data.oppositeVertexIndex));
        auto vertex2 = childNode->data.faceHandle->vertex(childNode->data.faceHandle->ccw(childNode->data.oppositeVertexIndex));


        cost.vertex1 = vertex1;
        cost.vertex2 = vertex2;
        
        width = hypot(vertex1->point().x() - vertex2->point().x(), vertex1->point().y() - vertex2->point().y());

        if ((vertex1->info().color == vertex2->info().color) && vertex1->info().color != 0) {
            coneColor = _coneColor;
        }

        if ((vertex1->info().color == vertex2->info().color) && vertex1->info().color == 0) {
            coneNoColor = _coneNoColor;
        }

        if(childNode->parent->parent != NULL){

            while(grand_parent->parent != NULL) {
                grand_parent = grand_parent->parent;
            }

            distance_child2GP = hypot((grand_parent->data.midpoint.pose.position.x - childNode->data.midpoint.pose.position.x), (grand_parent->data.midpoint.pose.position.y - childNode->data.midpoint.pose.position.y));
            distance_parent2GP = hypot((grand_parent->data.midpoint.pose.position.x - childNode->parent->data.midpoint.pose.position.x), (grand_parent->data.midpoint.pose.position.y - childNode->parent->data.midpoint.pose.position.y));

            //std::cout << "*****distance_child2PP = " << distance_child2PP << std::endl;
            //std::cout << "*****distance_parent2PP = " << distance_parent2PP << std::endl;

            if (distance_child2GP < distance_parent2GP) {
                going_back = 5000;
            }
        }

        /*if (vertex1->info().color != 0 && vertex2->info().color != 0 && vertex1->info().color != vertex2->info().color) {
        _coneColor = -10;
        }*/

        if (childNode->parent->parent == NULL) {
            u1 = 0.01;
            u2 = 0.0;
            v1 = 0.01 - childNode->data.midpoint.pose.position.x;
            v2 = 0.0 - childNode->data.midpoint.pose.position.y;

            angle = fabs(std::acos(-(u1 * v1 + u2 * v2) / (hypot(u1, u2) * hypot(v1, v2))));
        }
        else {
            u1 = childNode->parent->parent->data.midpoint.pose.position.x - childNode->parent->data.midpoint.pose.position.x;
            u2 = childNode->parent->parent->data.midpoint.pose.position.y - childNode->parent->data.midpoint.pose.position.y;
            v1 = childNode->parent->data.midpoint.pose.position.x - childNode->data.midpoint.pose.position.x;
            v2 = childNode->parent->data.midpoint.pose.position.y - childNode->data.midpoint.pose.position.y;

            angle = fabs(std::acos((u1 * v1 + u2 * v2) / (hypot(u1, u2) * hypot(v1, v2))));
         
        }
        if (angle == NAN) {
            angle = 0;
            exit(EXIT_FAILURE);
        }

        // vector from the parent midpoint to the child midpoint
        cone_coordinate1 = -v1;
        cone_coordinate2 = -v2;
        // vector from the parent midpoint to one of the cones on the side of the child midpoint
        cone_coordinate11 = vertex1->point().x() - childNode->parent->data.midpoint.pose.position.x;
        cone_coordinate12 = vertex1->point().y() - childNode->parent->data.midpoint.pose.position.y;
        // vector from the parent midpoint to the other cone on the side of the child midpoint
        cone_coordinate21 = vertex2->point().x() - childNode->parent->data.midpoint.pose.position.x;
        cone_coordinate22 = vertex2->point().y() - childNode->parent->data.midpoint.pose.position.y;

        angle_with_vertex1 = std::atan2(cone_coordinate2,cone_coordinate1) - std::atan2(cone_coordinate12,cone_coordinate11);
        if (angle_with_vertex1 >= 3.14 ) angle_with_vertex1 = 3.14 - angle_with_vertex1;
        if (angle_with_vertex1 <= -3.14 ) angle_with_vertex1 = -3.14 - angle_with_vertex1;

        angle_with_vertex2 = std::atan2(cone_coordinate2,cone_coordinate1) - std::atan2(cone_coordinate22,cone_coordinate21);
        if (angle_with_vertex2 >= 3.14) angle_with_vertex2 = 3.14 - angle_with_vertex2;
        if (angle_with_vertex2 <= -3.14) angle_with_vertex2 = -3.14 - angle_with_vertex2;


        //color = 1 -> blue 
        //color = 2 -> yellow

        if(angle_with_vertex1 < 0 && vertex1->info().color == 2){
            wrong_side = _wrongSide;
        }else if(angle_with_vertex1 > 0 && vertex1->info().color == 1){
           wrong_side = _wrongSide;
        }

        if(angle_with_vertex2 < 0 && vertex2->info().color == 2){
            wrong_side = _wrongSide;
        }else if(angle_with_vertex2 > 0 && vertex2->info().color == 1){
            wrong_side = _wrongSide;
        }

        /*std::cout << "\ncone_coordinate11 = " << cone_coordinate11 << std::endl;
        std::cout << "cone_coordinate12 = " << cone_coordinate12 << std::endl;
        std::cout << "cone_coordinate21 = " << cone_coordinate21 << std::endl;
        std::cout << "cone_coordinate22 = " << cone_coordinate22 << std::endl;
        std::cout << "v1 = " << v1 << std::endl;
        std::cout << "v2 = " << v2 << std::endl;
        std::cout << "u1 = " << u1 << std::endl;
        std::cout << "u2 = " << u2 << std::endl;
        std::cout << "angle_with_vertex1 = " << angle_with_vertex1 << std::endl;
        std::cout << "angle_with_vertex2 = " << angle_with_vertex2 << std::endl;
        std::cout << "color vertex1 = " << vertex1->info().color << std::endl;
        std::cout << "color vertex2 = " << vertex2->info().color << std::endl;*/

        cost.costParent = childNode->parent->data.cost.costTotal;
        cost.costWidthLeaf = ( _Kwidth*pow(width/_widthAdmMax,2) + _Kwidth*pow(width/_widthAdmMin,-2) );
        cost.costDistanceLeaf = _Kdistance * pow(distance / _distanceAdmMax, 2) + _Kdistance*pow(distance/_distanceAdmMin,-2);
        cost.costAngleLeaf = _Kangle * pow(angle / _angleAdm, 2);

        cost.costDistanceTotalLeaf = _Kdistance_total*pow(childNode->data.totalDistance/(childNode->data.pathSteps*_distanceTotalAdmMax), 2) + _Kdistance_total*pow(childNode->data.totalDistance/(childNode->data.pathSteps*_distanceTotalAdmMin), -2);
        
        // cost.costColor = _coneColor * vertex1->info().probability * vertex2->info().probability;
        cost.costColorLeaf = coneColor;
        cost.costNoColorLeaf = coneNoColor;
        cost.costGoingBackLeaf = going_back;
        cost.costWrongSideLeaf = wrong_side;

        cost.costParent = childNode->parent->data.cost.costTotal;
        cost.costTotal  = cost.costParent
                        + cost.costWidthLeaf
                        + cost.costDistanceLeaf
                        + cost.costAngleLeaf
                        + cost.costDistanceTotalLeaf
                        + cost.costColorLeaf
                        + cost.costNoColorLeaf
                        + cost.costGoingBackLeaf
                        + cost.costWrongSideLeaf;
        /*
        std::cout << "\n-------cost.costTotal = " << cost.costTotal << std::endl;
        std::cout << "-costParent = " << cost.costParent << std::endl;
        std::cout << "-costWidthLeaf = " << cost.costWidthLeaf << std::endl;
        std::cout << "-costDistanceLeaf = " << cost.costDistanceLeaf << std::endl;
        std::cout << "-costDistanceTotalLeaf = " << cost.costDistanceTotalLeaf << std::endl;
        std::cout << "-costAngleLeaf = " << cost.costAngleLeaf << std::endl;
        std::cout << "-costColorLeaf = " << cost.costColorLeaf << std::endl;
        std::cout << "-costNoColorLeaf = " << cost.costNoColorLeaf << std::endl;
        std::cout << "-costGoingBackLeaf = " << cost.costGoingBackLeaf << std::endl;
        std::cout << "-costWrongSideLeaf = " << cost.costWrongSideLeaf << std::endl; */

        cost.costWidth = childNode->parent->data.cost.costWidth + cost.costWidthLeaf;
        cost.costDistance = childNode->parent->data.cost.costDistance + cost.costDistanceLeaf;
        cost.costAngle = childNode->parent->data.cost.costAngle + cost.costAngleLeaf;
        cost.costDistanceTotal = childNode->parent->data.cost.costDistanceTotal + cost.costDistanceTotalLeaf;
        cost.costColor = childNode->parent->data.cost.costColor + cost.costColorLeaf;
        cost.costNoColor = childNode->parent->data.cost.costNoColor + cost.costNoColorLeaf;
        cost.costGoingBack = childNode->parent->data.cost.costGoingBack + cost.costGoingBackLeaf;
        cost.costWrongSide = childNode->parent->data.cost.costWrongSide + cost.costWrongSideLeaf;
        /*
        std::cout << ">>>>>>>cost.costTotal = " << cost.costWidth+cost.costDistance+cost.costAngle+cost.costDistanceTotal+cost.costColor+cost.costNoColor+cost.costGoingBack << std::endl;
        std::cout << ">cost.costWidth =  " << cost.costWidth << std::endl;
        std::cout << ">cost.costDistance = " << cost.costDistance << std::endl;
        std::cout << ">cost.costAngle = " << cost.costAngle << std::endl;
        std::cout << ">cost.costDistanceTotal = " << cost.costDistanceTotal << std::endl;
        std::cout << ">cost.costColor = " << cost.costColor << std::endl;
        std::cout << ">costNoColor = " << cost.costNoColor << std::endl;
        std::cout << ">costGoingBack = " << cost.costGoingBack << std::endl; */

    }
    return cost;
}


void PathPlannerTree::generateChildNodes(node *parentNode) {

    Delaunay::Face_handle newFace = parentNode->data.faceHandle->neighbor(parentNode->data.oppositeVertexIndex);

    if(_dt.is_infinite(newFace) || parentNode->data.pathSteps >= 10 || !keepTriangle(newFace)) {

        parentNode->data.cost.costTotal = parentNode->data.cost.costTotal / parentNode->data.pathSteps;
        parentNode->data.cost.costWidth = parentNode->data.cost.costWidth / parentNode->data.pathSteps;
        parentNode->data.cost.costDistance = parentNode->data.cost.costDistance / parentNode->data.pathSteps;
        parentNode->data.cost.costAngle = parentNode->data.cost.costAngle / parentNode->data.pathSteps;
        parentNode->data.cost.costDistanceTotal = parentNode->data.cost.costDistanceTotal / parentNode->data.pathSteps;
        parentNode->data.cost.costColor = parentNode->data.cost.costColor / parentNode->data.pathSteps;
        parentNode->data.cost.costNoColor = parentNode->data.cost.costNoColor / parentNode->data.pathSteps;
        parentNode->data.cost.costGoingBack = parentNode->data.cost.costGoingBack / parentNode->data.pathSteps;
        parentNode->data.cost.costWrongSide = parentNode->data.cost.costWrongSide / parentNode->data.pathSteps;

        _leafnode.push_back(parentNode);
        return;
    }

	int nextFaceVerticeIndex = _dt.mirror_index(parentNode->data.faceHandle, parentNode->data.oppositeVertexIndex);  
    Delaunay::Vertex_handle oppositeVertex = _dt.mirror_vertex(parentNode->data.faceHandle, parentNode->data.oppositeVertexIndex); 

    struct node *childNode1 = new node;
    struct node *childNode2 = new node;

    parentNode->leftChild = childNode1;
    parentNode->rightChild = childNode2;

    childNode1->data.faceHandle = newFace;
    childNode1->data.oppositeVertexIndex = (newFace->ccw(nextFaceVerticeIndex));
    childNode1->parent = parentNode;
    childNode1->data.pathSteps = parentNode->data.pathSteps + 1;
    childNode1->data.midpoint = calculateMidpoint(newFace->vertex(nextFaceVerticeIndex), newFace->vertex(newFace->cw(nextFaceVerticeIndex)));		
    childNode1->data.cost = calculateCost(childNode1);
    checkLoop(childNode1);
    _generatedList.push_back(childNode1);
    
    childNode2->data.faceHandle = newFace;
    childNode2->data.oppositeVertexIndex = (newFace->cw(nextFaceVerticeIndex));
    childNode2->parent = parentNode;
    childNode2->data.pathSteps = parentNode->data.pathSteps + 1;
    childNode2->data.midpoint = calculateMidpoint(newFace->vertex(nextFaceVerticeIndex), newFace->vertex(newFace->ccw(nextFaceVerticeIndex)));
    childNode2->data.cost = calculateCost(childNode2);
    checkLoop(childNode2);
    _generatedList.push_back(childNode2);
    
}

void PathPlannerTree::checkLoop (node *newNode) {
    std::vector<node*>::iterator it = std::find_if(_generatedList.begin(),_generatedList.end(), [&] (const node* element) {
        return (element->data.midpoint.pose.position.x == newNode->data.midpoint.pose.position.x && element->data.midpoint.pose.position.y == newNode->data.midpoint.pose.position.y);
        });

    if (it == _generatedList.end()) {
        _queue.push(newNode);
        return;
    }        

    if (newNode->data.cost.costTotal > (*it)->data.cost.costTotal) {
        return;
    }
    else {
        _queue.push(newNode);
    }
}


void PathPlannerTree::findTrajectory() {
    node *temp_best;
    node *best = new node;
    node *second_best = new node;
    node *third_best = new node;
    best->data.cost.costTotal = INFINITY;
    second_best->data.cost.costTotal = INFINITY;
    third_best->data.cost.costTotal = INFINITY;
    
    for (const auto leafNodeIt: _leafnode) {
        if (leafNodeIt->data.cost.costTotal < best->data.cost.costTotal) {
            third_best = second_best;
            second_best = best;
            best = leafNodeIt;
        }
        else if (leafNodeIt->data.cost.costTotal < second_best->data.cost.costTotal) {
            third_best = second_best;
            second_best = leafNodeIt;
        }
        else if (leafNodeIt->data.cost.costTotal < third_best->data.cost.costTotal) {
            third_best = leafNodeIt;
        }
    }
    /*
    std::cout << "Best total cost: " << best->data.cost.costTotal << std::endl;
    std::cout << "vertice1: " << best->data.cost.vertex1->info().color << std::endl;
    std::cout << "vertice2: " << best->data.cost.vertex2->info().color << std::endl;

    std::cout << "Second best total cost: " << second_best->data.cost.costTotal << std::endl;
    std::cout << "Third best total cost: " << third_best->data.cost.costTotal << std::endl; */

    temp_best = best;

    while(temp_best != NULL) {
        _bestTrajectory.poses.push_back(temp_best->data.midpoint);
        temp_best = temp_best->parent;
    }

    _bestCost.header.stamp = ros::Time::now();
    _bestCost.costTotal = best->data.cost.costTotal;
    _bestCost.costWidth = best->data.cost.costWidth;
    _bestCost.costDistance = best->data.cost.costDistance;
    _bestCost.costAngle = best->data.cost.costAngle;
    _bestCost.costDistanceTotal = best->data.cost.costDistanceTotal;
    _bestCost.costColor = best->data.cost.costColor;
    _bestCost.costNoColor = best->data.cost.costNoColor;
    _bestCost.costGoingBack = best->data.cost.costGoingBack;
    _bestCost.costWrongSide = best->data.cost.costWrongSide;

    _secondBestCost.header.stamp = ros::Time::now();
    _secondBestCost.costTotal = second_best->data.cost.costTotal;
    _secondBestCost.costWidth = second_best->data.cost.costWidth;
    _secondBestCost.costDistance = second_best->data.cost.costDistance;
    _secondBestCost.costAngle = second_best->data.cost.costAngle;
    _secondBestCost.costDistanceTotal = second_best->data.cost.costDistanceTotal;
    _secondBestCost.costColor = second_best->data.cost.costColor;
    _secondBestCost.costNoColor = best->data.cost.costNoColor;
    _secondBestCost.costGoingBack = best->data.cost.costGoingBack;
    _secondBestCost.costWrongSide = best->data.cost.costWrongSide;

    _thirdBestCost.header.stamp = ros::Time::now();
    _thirdBestCost.costTotal = third_best->data.cost.costTotal;
    _thirdBestCost.costWidth = third_best->data.cost.costWidth;
    _thirdBestCost.costDistance = third_best->data.cost.costDistance;
    _thirdBestCost.costAngle = third_best->data.cost.costAngle;
    _thirdBestCost.costDistanceTotal = third_best->data.cost.costDistanceTotal;
    _thirdBestCost.costColor = third_best->data.cost.costColor;
    _thirdBestCost.costNoColor = best->data.cost.costNoColor;
    _thirdBestCost.costGoingBack = best->data.cost.costGoingBack;
    _thirdBestCost.costWrongSide = best->data.cost.costWrongSide;

    reverse(_bestTrajectory.poses.begin(), _bestTrajectory.poses.end());
}



/**********************************************************************
****** VIZUALIZATION ON RVIZ METHODS AND TO PRINT TREE IN PROMPT ******
**********************************************************************/

void PathPlannerTree::vizualization() {
    vizualizeDelaunay();
    vizualizeChosenMidpoints();
    vizualizeAllPaths();
}

void PathPlannerTree::vizualizeDelaunay() {

    geometry_msgs::Point delaunayEdgePoint1;
    geometry_msgs::Point delaunayEdgePoint2;

    _delaunayMarkerLines.ns = "Delaunay edges";
    _delaunayMarkerLines.type = visualization_msgs::Marker::LINE_LIST;
    _delaunayMarkerLines.scale.x = 0.05;
    _delaunayMarkerLines.color.a = 1.0;
    _delaunayMarkerLines.color.r = 0.0;
    _delaunayMarkerLines.color.g = 0.0;
    _delaunayMarkerLines.color.b = 0.0;

    delaunayEdgePoint1.z = 0;
    delaunayEdgePoint2.z = 0;

    for(auto edgesIt = _dt.finite_edges_begin(); edgesIt != _dt.finite_edges_end(); ++edgesIt) {        
        delaunayEdgePoint1.x = edgesIt->first->vertex(edgesIt->first->cw(edgesIt->second))->point().x();
        delaunayEdgePoint1.y = edgesIt->first->vertex(edgesIt->first->cw(edgesIt->second))->point().y();
        delaunayEdgePoint2.x = edgesIt->first->vertex(edgesIt->first->ccw(edgesIt->second))->point().x();
        delaunayEdgePoint2.y = edgesIt->first->vertex(edgesIt->first->ccw(edgesIt->second))->point().y();
    
        _delaunayMarkerLines.points.push_back(delaunayEdgePoint1);
        _delaunayMarkerLines.points.push_back(delaunayEdgePoint2);
    }
}

void PathPlannerTree::vizualizeChosenMidpoints() {

    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker marker;
    int i = 0;

    markers.markers.clear();

    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.ns = "Delaunay midpoints";
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.lifetime = ros::Duration(0.1);

    for (auto it = _bestTrajectory.poses.begin(); it != _bestTrajectory.poses.end(); ++it, i++) {        
        marker.id = i;  
        marker.pose.position.x = (*it).pose.position.x;
        marker.pose.position.y = (*it).pose.position.y;
        marker.pose.position.z = (*it).pose.position.z;
        markers.markers.push_back(marker);
    }
    _bestTrajectoryMarkers = markers;
}

void PathPlannerTree::vizualizeAllPaths() {
    
    _midpointConnectionsLines.ns = "midpointsConnections";
    _midpointConnectionsLines.type = visualization_msgs::Marker::LINE_LIST;
    _midpointConnectionsLines.scale.x = 0.02;
    _midpointConnectionsLines.color.a = 1.0;
    _midpointConnectionsLines.color.r = 0.0;
    _midpointConnectionsLines.color.g = 0.2;
    _midpointConnectionsLines.color.b = 1.0;   
    _midpointConnectionsLines.color.b = 1.0;      
    _midpointConnectionsLines.color.b = 1.0;   
    _midpointConnectionsLines.color.b = 1.0;      
    _midpointConnectionsLines.color.b = 1.0;   
    _midpointConnectionsLines.color.b = 1.0;      
    _midpointConnectionsLines.color.b = 1.0;   

    traverseTree(_root, 0);
} 

// Function to print binary tree in 2D  
// It does reverse inorder traversal  
void PathPlannerTree::traverseTree(node *root, int space) {  

    geometry_msgs::Point pointOfPossiblePath;
    int count = 10;
    
    // Base case  
    if (root == NULL)  
        return;
    else if (root == _root) {
        for (auto &parentChild: _root->parentChilds) {
            traverseTree(parentChild, count);    
        }
    }
 
  
   /** UNCOMMENT IF YOU WANT TO PRINT TREE **/
    // space += count;  
  
    // Process right child first  
    traverseTree(root->rightChild, space);  
  
    /** UNCOMMENT IF YOU WANT TO PRINT TREE **/
    // std::cout << std::endl;  
    // for (int i = count; i < space; i++)  
    //     std::cout << " ";
    // std::cout << "(" << root->data.midpoint.pose.position.x << ", " << root->data.midpoint.pose.position.y << ") - " << root->data.cost.costTotal << std::endl;  

    //add line pair
    if(root->parent != NULL && root->parent != _root) {
        pointOfPossiblePath.x = root->parent->data.midpoint.pose.position.x;
        pointOfPossiblePath.y = root->parent->data.midpoint.pose.position.y;
        pointOfPossiblePath.z = 0;
        _midpointConnectionsLines.points.push_back(pointOfPossiblePath);
        pointOfPossiblePath.x = root->data.midpoint.pose.position.x;
        pointOfPossiblePath.y = root->data.midpoint.pose.position.y;
        pointOfPossiblePath.z = 0;
        _midpointConnectionsLines.points.push_back(pointOfPossiblePath);
    }
    // Process left child  
    traverseTree(root->leftChild, space);  
} 