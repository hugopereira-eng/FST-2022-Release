#ifndef SERVER_HPP
#define SERVER_HPP

#define MAXLINE 1024

#include <stdio.h>
#include <signal.h>
#include <stdlib.h>
#include <string>
#include <netinet/in.h>
#include <sys/socket.h>
#include <stdio.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include "lib/json/json.h"
#include "lib/fcp-cpp/candata.h"


typedef struct _can_line_info {
    uint16_t can_fd;
    uint16_t sniffer_fd;
    uint16_t PORT;
    std::string dev_name;
} can_line_info;

void* vsniffer_to_vcan(void*);
int connect_to_CAN(const char* ifname);
void* vcan_to_vsniffer(void* index);
void CANdata_to_CANFrame(CANdata message, struct can_frame* frame);
int send_to_CAN_Bus_Line(int sockfd, struct can_frame frame);
int receive_from_CAN_Bus_Line(int sockfd, struct can_frame* frame);
void CANFrame_to_CANdata(struct can_frame frame, CANdata* message);
#endif