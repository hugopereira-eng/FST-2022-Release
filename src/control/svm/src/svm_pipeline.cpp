#include "svm/svm_pipeline.hpp"

// Constructor
SVM::SVM() {
	if(!ros::param::get("common/mission_selected", _mission)) {
		ROS_WARN_STREAM("Could not load mission. Trackdrive will be assumed");
		_mission = TRACKDRIVE;
	}
}

// Getters
nav_msgs::Path const & SVM::getPath() const { return _centerLine; }
sensor_msgs::Image const & SVM::getSvmImage() const { return _svmImage; }

void SVM::setConeDetectionsSlam(const common_msgs::ConeDetections &newConeDetections) {
	_coneDetections = newConeDetections;
}

void SVM::setConeDetectionsSensorFusion(const common_msgs::ConeDetections &newConeDetections) {
	_coneDetections = newConeDetections;
}

void SVM::runAlgorithm() {
	if (_svmModel.empty()) setupModel();
	findCenterLine();
}

void SVM::findCenterLine() {

	int numCones = 0;
    std::vector<float*> conesVec;
	std::vector<float*> conesInImageVec;
	std::vector<int> labelsVec;
	bool blueFlag = false, yellowFlag = false;

	int imgTransf = 6;
	int imgXOffset = 2;
	float* currentCone;
	float* currentCone2;	
	float maxX, maxY, minY;

	if (!_coneDetections.cone_detections.empty()) {
		auto minMaxYIter = std::minmax_element(_coneDetections.cone_detections.begin(), _coneDetections.cone_detections.end(),
			[](const common_msgs::Cone &a, const common_msgs::Cone &b) {	
			return a.position.y < b.position.y; });
		minY = minMaxYIter.first->position.y;
		maxY = minMaxYIter.second->position.y;

		auto maxXIter = std::max_element(_coneDetections.cone_detections.begin(), _coneDetections.cone_detections.end(),
			[](const common_msgs::Cone &a, const common_msgs::Cone &b) {	
			return a.position.x < b.position.x; });
		
		maxX =  maxXIter->position.x;
	}
	
	float imgYsize = abs(minY) * imgTransf + abs(maxY) *imgTransf;
	float imgXsize = abs(maxX) * imgTransf + 20;
	float imgYOffset = imgYsize * ((abs(minY)*imgTransf)/imgYsize) + 10;

	for (const auto &cone: _coneDetections.cone_detections) {
		if (_mission == ACCELERATION){
			if (cone.position.y > 0 && cone.position.x < 15) {
				labelsVec.push_back(-1);
				blueFlag = true;
			} else if(cone.position.y < 0 && cone.position.x < 15){
				labelsVec.push_back(1);
				yellowFlag = true;	
			} else {
				continue;
			}
		} else {
			
			if (cone.color == BLUE_CONE) {
				labelsVec.push_back(-1);
				blueFlag = true;
			} else if (cone.color == YELLOW_CONE) {
				labelsVec.push_back(1);
				yellowFlag = true;
			} else {
				continue;
			}

		}

		currentCone = (float*) malloc(sizeof(float) * 2);
		currentCone[0] = cone.position.x;
		currentCone[1] = cone.position.y;
		conesVec.push_back(currentCone);
		currentCone2 = (float*) malloc(sizeof(float) * 2);
		currentCone2[0] = cone.position.x * imgTransf + imgXOffset;
		currentCone2[1] = cone.position.y * imgTransf + imgYOffset;
		conesInImageVec.push_back(currentCone2);
		numCones++;
	}

	// If there are no yellow or no blue cones use the previous path
	if (!blueFlag && !yellowFlag) {
		_centerLine = _lastPath;
		return;
	}

	// Add blue cone to left of the car
	currentCone = (float*) malloc(sizeof(float) * 2);
	currentCone[0] = 0.1;
	currentCone[1] = 1.75;
	conesVec.push_back(currentCone);
	currentCone2 = (float*) malloc(sizeof(float) * 2);
	currentCone2[0] = 0.1 * imgTransf;
	currentCone2[1] = 1.75 * imgTransf + imgYOffset;
	conesInImageVec.push_back(currentCone2);
	numCones++;
	labelsVec.push_back(-1); // blue cone

	// Add yellow cone to right of the car
	currentCone = (float*) malloc(sizeof(float) * 2);
	currentCone[0] = 0.1;
	currentCone[1] = -1.75;
	conesVec.push_back(currentCone);
	currentCone2 = (float*) malloc(sizeof(float) * 2);
	currentCone2[0] = 0.1 * imgTransf;
	currentCone2[1] = -1.75 * imgTransf + imgYOffset;
	conesInImageVec.push_back(currentCone2);
	numCones++;
	labelsVec.push_back(1); // yellow cone

	float** conesInImage = &conesInImageVec[0];
    int* labels = &labelsVec[0];
	float conesInImage2[numCones][2];
	std::vector<float> xVector;
	std::vector<float> yVector;
	
	for (int i = 0; i < numCones; i++) {
		conesInImage2[i][0]= conesInImage[i][0];
		conesInImage2[i][1]= conesInImage[i][1];
		xVector.push_back(conesInImage2[i][0]);
		yVector.push_back(conesInImage2[i][1]);
	}

	// Convert cones data to feed the SVM
    cv::Mat conesMat(numCones, 2, CV_32F, conesInImage2);
	cv::Mat labelsMat(numCones, 1, CV_32SC1, labels);

	// Train the model
	_svmModel->train(conesMat, cv::ml::ROW_SAMPLE, labelsMat);

	// Data for representation
    int width = imgXsize, height = imgYsize + 20;
    cv::Mat image = cv::Mat::zeros(height, width, CV_8UC3);
	cv::Scalar color;

    // Show the decision regions given by the SVM
    cv::Vec3b black(0,0,0), white(255,255,255);
    for (int i = 0; i < image.rows; i++) {
        for (int j = 0; j < image.cols; j++) {
            cv::Mat sampleMat = (cv::Mat_<float>(1,2) << j,i);
            float response = _svmModel->predict(sampleMat);
            if (response == 1)
                image.at<cv::Vec3b>(i,j) = black;
            else if (response == -1)
                image.at<cv::Vec3b>(i,j) = white;
        }
    }

	// Create image for the contour
    std::vector<std::vector<cv::Point> > contours;
    cv::Mat contourOutput;
	cvtColor(image, contourOutput, CV_BGR2GRAY); // tem de ser CV_8C1

	// Get the edges of the centerline
	cv::Canny (contourOutput,contourOutput,255/3,255,3);
	std::vector<cv::Point> nz; 
	findNonZero(contourOutput,nz);

	if(nz.empty()){
		_centerLine = _lastPath;
		return;
	}

	_centerLine.poses.erase(_centerLine.poses.begin(),_centerLine.poses.end());
	_centerLine.header = _coneDetections.header;
	nav_msgs::Path pathAux;
	pathAux.header = _coneDetections.header;
	pathAux.poses.erase(pathAux.poses.begin(),pathAux.poses.end());

	for (int i =0; i < nz.size(); i++){
		geometry_msgs::PoseStamped centerLinePoint;
		centerLinePoint.pose.position.x = (float(nz.at(i).x)- imgXOffset) / imgTransf;
		centerLinePoint.pose.position.y = (float(nz.at(i).y) - imgYOffset) / imgTransf;
		
		pathAux.poses.push_back(centerLinePoint);
	}
	
	std::sort(pathAux.poses.begin(), pathAux.poses.end(),
        [](const geometry_msgs::PoseStamped &a, const geometry_msgs::PoseStamped &b) -> bool {	
			const double normA = std::sqrt((std::pow(a.pose.position.x, 2) + std::pow(a.pose.position.y, 2)));
			const double normB = std::sqrt((std::pow(b.pose.position.x, 2) + std::pow(b.pose.position.y, 2)));
			return normA < normB; });
	
	_centerLine.poses.reserve(pathAux.poses.size());
	_centerLine = pathAux;
	

	// -------------Visualization ------------------------
	// cv::Mat outcontour;
	// cv::resize(contourOutput, outcontour, cv::Size(), 4.5, 4.5);
	// cv::imshow("contour", outcontour); // show it to the user
    // cv::waitKey(1);

	for (int j = 0; j < numCones; j++) {
		if (labels[j] == -1) {
			color = cv::Scalar(255, 108, 0);
		}
		if (labels[j] == 1) {
			color = cv::Scalar(0, 255, 216);
		}
		cv::circle(image, cv::Point(conesInImage[j][0], conesInImage[j][1]), 1, color, -1);
	}
	cv::Mat outImg;
	cv::resize(image, outImg, cv::Size(), 4.5, 4.5);
	cv_bridge::CvImage cvImage = cv_bridge::CvImage(_coneDetections.header, sensor_msgs::image_encodings::BGR8, outImg);
	cvImage.toImageMsg(_svmImage);
  
    // cv::imshow("SVM Simple Example", outImg); // show it to the user
	// cv::waitKey(1);

	// -----------------------------------------------

	// Path smoothing
	for (int j = 0; j < 10; j++) {
		for (int i = 1; i < _centerLine.poses.size() - 1; i++) {
			_centerLine.poses[i].pose.position.x = (_centerLine.poses[i-1].pose.position.x + _centerLine.poses[i+1].pose.position.x) / 2;
			_centerLine.poses[i].pose.position.y = (_centerLine.poses[i-1].pose.position.y + _centerLine.poses[i+1].pose.position.y) / 2;
		}
	}

	// Save path in case next iteration there are no yellow or blue cones
	_lastPath = _centerLine;    
}

void SVM::setupModel() {
	ROS_INFO("SVM Setup Model");
	_svmModel = cv::ml::SVM::create();
    _svmModel->setType(cv::ml::SVM::C_SVC);
    _svmModel->setC(100);
	_svmModel->setGamma(0.0001);
	// if(_mission == ACCELERATION){
    // 	_svmModel->setKernel(cv::ml::SVM::LINEAR);
	// } else {
    _svmModel->setKernel(cv::ml::SVM::RBF);
	// }
    _svmModel->setTermCriteria(cv::TermCriteria(cv::TermCriteria::MAX_ITER, 100, 1e-6));
	ROS_INFO("SVM Model Ready");
}