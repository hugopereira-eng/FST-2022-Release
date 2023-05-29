#ifndef PATH_PLANNER_PATH_TREE_HPP
#define PATH_PLANNER_PATH_TREE_HPP

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/Projection_traits_xy_3.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include "nav_msgs/Path.h"
#include <visualization_msgs/MarkerArray.h>
#include <queue>
#include <algorithm>
#include <math.h>
#include "common_msgs/Cost.h"

typedef struct colorData {
    int color;
    // float probability;
} ColorData;

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Triangulation_vertex_base_with_info_2<ColorData, K> Vb;
typedef CGAL::Triangulation_data_structure_2<Vb> Tds;
typedef CGAL::Delaunay_triangulation_2<K, Tds> Delaunay;
typedef Delaunay::Face_handle Face_handle;
typedef K::Point_2 Point;

#define PI 3.14159265

// cost function parcels
typedef struct costData {
    float costTotal = 0;
    float costWidthLeaf = 0;            // Cost associated with the track width above 4,5m and below 2,5m
    float costParent = 0;           // Total cost of the parent node
    float costDistanceLeaf = 0;         // Midpoint distance from parent to child
    float costAngleLeaf = 0;            // Angle from the parent direction to the child direction
    float costDistanceTotalLeaf = 0;    // Sum of all the midpoint distances
    float costColorLeaf = 0;
    float costGoingBackLeaf = 0;
    float costNoColorLeaf = 0;
    float costWrongSideLeaf = 0;

    float costWidth = 0;
    float costDistance = 0;
    float costAngle = 0;
    float costDistanceTotal = 0;
    float costColor = 0;
    float costGoingBack = 0;
    float costNoColor = 0;
    float costWrongSide = 0;

    Tds::Vertex_handle vertex1;
    Tds::Vertex_handle vertex2;

} CostData;


// data of a node
typedef struct treeData {
    int oppositeVertexIndex;  
    Face_handle faceHandle;
    geometry_msgs::PoseStamped midpoint;
    CostData cost;
    float totalDistance;
    int pathSteps;
} TreeData;

// tree node
struct node {
    TreeData data;
    node *parent = NULL;
    node *leftChild = NULL;
    node *rightChild = NULL;
    std::vector<node *> parentChilds; // hacky thing to allow the root of three to have more than two childNodes
};

class PathPlannerTree {
public:    
    PathPlannerTree();

    nav_msgs::Path const &getBestPath() const;
    visualization_msgs::MarkerArray const &getBestTrajectoryMarkers() const;
    visualization_msgs::Marker const & getDelaunayMarkerLines() const;
    visualization_msgs::Marker const & getMidpointConnectionsLines() const;
    common_msgs::Cost const & getCostTree() const;
    common_msgs::Cost const & getBestCost() const;
    common_msgs::Cost const & getSecondBestCost() const; 
    common_msgs::Cost const & getThirdBestCost() const; 

    void setKwidth(double Kwidth);
	void setKdistance(double Kdistance);
	void setKangle(double Kangle);
	void setKdistance_total(double Kdistance_total);
	void setconeColor(double coneColor);
    void setconeNoColor(double coneNoColor);
    void setwrongSide(double wrongSide);

    void setWidthAdmMax(double widthAdmMax);
    void setWidthAdmMin(double widthAdmMin);
    void setDistanceAdmMax(double distanceAdmMax);
    void setDistanceAdmMin(double distanceAdmMin);
    void setAngleAdm(double angleAdm);
    void setDistanceTotalAdmMax(double distanceTotalAdmMax);
    void setDistanceTotalAdmMin(double distanceTotalAdmMin);
    
    ~PathPlannerTree();
    void generateTree(Delaunay &dt); //ou sem &??
    void findTrajectory();
    void deleteAllTree();
    void vizualization();

private:

    nav_msgs::Path _bestTrajectory;
    visualization_msgs::MarkerArray _bestTrajectoryMarkers;
    visualization_msgs::Marker _delaunayMarkerLines;
    visualization_msgs::Marker _midpointConnectionsLines;
   
    Delaunay _dt;
    node* _root = NULL;
    std::queue<node*> _queue;
    std::vector<node*> _leafnode;
    std::vector<node*> _generatedList;
    int _maxSteps = 5;

    common_msgs::Cost _bestCost;
    common_msgs::Cost _secondBestCost;
    common_msgs::Cost _thirdBestCost;
    
    void generateFirstElement();
    void generateChildNodes(node *node);
    geometry_msgs::PoseStamped calculateMidpoint(Delaunay::Vertex_handle v1, Delaunay::Vertex_handle v2);
    CostData calculateCost(node *node);
    costData SumParcels(node *childNode,costData cost);
    void checkLoop(node *newNode);
    void deleteTree(node *root);
   
    void cleanDataStructures();
    bool keepTriangle(Delaunay::Face_handle face);
    void vizualizeDelaunay();
    void vizualizeAllPaths();
    void traverseTree(node *root, int space);
    void vizualizeChosenMidpoints();

    float _coneColor;
    float _Kdistance;
    float _Kangle;
    float _Kwidth;
    float _Kdistance_total;
    float _coneNoColor;
    float _wrongSide;

    float _widthAdmMax;
    float _widthAdmMin;
    float _distanceAdmMax;
    float _distanceAdmMin;
    float _angleAdm;
    float _distanceTotalAdmMax;
    float _distanceTotalAdmMin;

};

#endif
