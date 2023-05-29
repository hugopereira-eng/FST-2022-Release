#include "launcher/launcher_handle.h"
#include "common_msgs/Mission.h"
#include <cstdlib>
using namespace std;
#include<string.h>

LauncherHandle::LauncherHandle(ros::NodeHandle &nodeHandle) : _nodeHandle(nodeHandle) {
    subscribeToTopics();
    _mission = MANUAL_DRIVING;
    _oldMission = MANUAL_DRIVING;
    //launcher();
}

//Getter
//common_msgs::Mission LauncherHandle::getMission() const { return _mission; }

// Subscribe to missionTopic
void LauncherHandle::subscribeToTopics() {
  ROS_INFO("subscribe to topics");
  _missionSubscriber = _nodeHandle.subscribe("/missionTopic", 1, &LauncherHandle::missionCallback, this);
}

void LauncherHandle::missionCallback(const common_msgs::Mission &mission) {
    //ROS_INFO("CALLBACK! updating mission message");
    _mission = (as_missions)mission.mission;
    launcher();
}

void LauncherHandle::launcher() {
    int systemKillRet;
    int systemLaunchRet;

    if(_oldMission == _mission){}
    else{
        switch (_mission){
            case ACCELERATION:
            
                systemKillRet = system("rosnode kill /mission_tracker");

                if(systemKillRet != -1){
                    cout << endl << "Mission stopped successfully!" << endl;
                }
                sleep(5);
                
                systemLaunchRet = system("roslaunch common_meta_pkg acceleration.launch &");

                if(systemLaunchRet != -1){
                    cout << endl << "Acceleration launched successfully!" << endl;
                }
                sleep(5);
                _oldMission = ACCELERATION;
                break;

            case SKIDPAD:

                systemKillRet = system("rosnode kill /mission_tracker");

                if(systemKillRet != -1){
                    cout << endl << "Mission stopped successfully!" << endl;
                }
                sleep(5);

                systemLaunchRet = system("roslaunch common_meta_pkg skidpad.launch & ");

                if(systemLaunchRet != -1){
                    cout << endl << "Skidpad launched successfully!" << endl;
                }
                sleep(5);
                _oldMission = SKIDPAD;
                break;
            
            case AUTOCROSS:

                systemKillRet = system("rosnode kill /mission_tracker");

                if(systemKillRet != -1){
                    cout << endl << "Mission stopped successfully!" << endl;
                }
                sleep(5);
                
                systemLaunchRet = system("roslaunch common_meta_pkg autocross.launch &");
                if(systemLaunchRet != -1){
                    cout << endl << "Autocross launched successfully!" << endl;
                }
                sleep(5);
                _oldMission = AUTOCROSS;
                break;
            
            case TRACKDRIVE:
                
                systemKillRet = system("rosnode kill /mission_tracker");

                if(systemKillRet != -1){
                    cout << endl << "Mission stopped successfully!" << endl;
                }
                sleep(5);

                systemLaunchRet = system("roslaunch common_meta_pkg trackdrive.launch &");
                if(systemLaunchRet != -1){
                    cout << endl << "Trackdrive launched successfully!" << endl;
                }
                sleep(5);
                _oldMission = TRACKDRIVE;
                break;

            case EBS_TEST:

                systemKillRet = system("rosnode kill /mission_tracker");

                if(systemKillRet != -1){
                    cout << endl << "Mission stopped successfully!" << endl;
                }
                sleep(5);
                systemLaunchRet = system("roslaunch common_meta_pkg ebs.launch &");
                if(systemLaunchRet != -1){
                    cout << endl << "Ebs Mission launched successfully!" << endl;
                }
                sleep(5);
                _oldMission = EBS_TEST;
                break;
            
            case INSPECTION:
            
                systemKillRet = system("rosnode kill /mission_tracker");

                if(systemKillRet != -1){
                    cout << endl << "Mission stopped successfully!" << endl;
                }
                sleep(5);
                systemLaunchRet = system("roslaunch common_meta_pkg inspection.launch &");
                if(systemLaunchRet != -1){
                    cout << endl << "Inspection Mission launched successfully!" << endl;
                }
                sleep(5);
                _oldMission = INSPECTION;
                break;

            case MANUAL_DRIVING:
                systemKillRet = system("rosnode kill /mission_tracker");
                _oldMission = MANUAL_DRIVING;
                break;
            
            case DARKNET_RECORD:
            
                systemKillRet = system("rosnode kill /mission_tracker");

                if(systemKillRet != -1){
                    cout << endl << "Mission stopped successfully!" << endl;
                }
                sleep(5);
                systemLaunchRet = system("roslaunch common_meta_pkg record_darknet.launch &");
                if(systemLaunchRet != -1){
                    cout << endl << "Darknet Recording Mission launched successfully!" << endl;
                }
                sleep(5);
                _oldMission = DARKNET_RECORD;
                break;

            case COLOR_CLASS_RECORD:
            
                systemKillRet = system("rosnode kill /mission_tracker");

                if(systemKillRet != -1){
                    cout << endl << "Mission stopped successfully!" << endl;
                }
                sleep(5);
                systemLaunchRet = system("roslaunch common_meta_pkg record_color_classification.launch &");
                if(systemLaunchRet != -1){
                    cout << endl << "Color Classification Recording Mission launched successfully!" << endl;
                }
                sleep(5);
                _oldMission = COLOR_CLASS_RECORD;
                break;

            case DARKNET_COLOR_CLASS_RECORD:
            
                systemKillRet = system("rosnode kill /mission_tracker");

                if(systemKillRet != -1){
                    cout << endl << "Mission stopped successfully!" << endl;
                }
                sleep(5);
                systemLaunchRet = system("roslaunch common_meta_pkg record_darknet_color_classification.launch &");
                if(systemLaunchRet != -1){
                    cout << endl << "Darknet and Color Classification Recording Mission launched successfully!" << endl;
                }
                sleep(5);
                _oldMission = DARKNET_COLOR_CLASS_RECORD;
                break;
        }
    }  
}