#include "can_sniffer/can_sniffer_handle.hpp"
#include <chrono>
#include <boost/thread.hpp>

boost::thread t1, t2;

void sigint (int sig){
    printf("Crtl+C received\n");
    pthread_kill(t1.native_handle(), 9);
    pthread_kill(t2.native_handle(), 9);
    ros::shutdown();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "can_sniffer");
    ros::NodeHandle nh("~");

    signal(SIGINT, sigint);

    can_line_info essential, sensors, canopen;

    bool simul = nh.param<bool>("simulation", false);
    if (simul){
        essential.dev_name  = "vcan0"; essential.PORT = 57005;
        sensors.dev_name    = "vcan1"; sensors.PORT   = 48879;
        canopen.dev_name    = "vcan2";
    }else {
        essential.dev_name  = "can0";
        sensors.dev_name    = "can1";
        canopen.dev_name    = "can2";
    }

    essential.can_fd = connect_to_CAN(essential.dev_name.c_str());
    sensors.can_fd   = connect_to_CAN(sensors.dev_name.c_str());
    canopen.can_fd   = connect_to_CAN(canopen.dev_name.c_str());

    CanSnifferHandle canSnifferHandle(nh, essential.can_fd, sensors.can_fd, canopen.can_fd);

    if(simul) {
        std::cout << "THREADS CREATED" << std::endl;
        t1 = boost::thread(boost::bind(&vsniffer_to_vcan, &essential));
        t2 = boost::thread(boost::bind(&vcan_to_vsniffer, &essential));
    }

    //ros::Rate loop_rate(64000);
    while (ros::ok()) {
        canSnifferHandle.run();
        ros::spinOnce(); 
        //loop_rate.sleep(); // Sleep for loop_rate   
    }

    return 0;
}
