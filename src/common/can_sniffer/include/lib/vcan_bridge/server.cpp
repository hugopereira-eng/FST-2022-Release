#include "server.hpp"

#define GET_DEV_ID(sid) ((sid) & 0x1F)
#define GET_MSG_ID(sid) (((sid) >> 5) & 0x3F)

int connect_to_CAN(const char* ifname) {
	/*See SocketCan*/
   	int cansockfd;
	struct sockaddr_can addr;
	struct ifreq ifr;

	cansockfd = socket(PF_CAN, SOCK_RAW, CAN_RAW);

	// int size = 0;

	// setsockopt(cansockfd, SOL_SOCKET, SO_RCVBUF, &size, sizeof(size));

	strcpy(ifr.ifr_name, ifname);
	ioctl(cansockfd, SIOCGIFINDEX, &ifr);

	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	bind(cansockfd, (struct sockaddr * ) &addr, sizeof(addr));

	return cansockfd;
}

void* vsniffer_to_vcan(void* arg) {

	can_line_info *info = (can_line_info *)arg;
	struct sockaddr_in servaddr;

	if ( (info->sniffer_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

    memset(&servaddr, 0, sizeof(servaddr));

    // Filling server information
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(info->PORT);
    servaddr.sin_addr.s_addr = INADDR_ANY;

    unsigned int nbytes, len = sizeof(servaddr);
	char *hello = (char *) "{\"sid\": 0, \"dlc\": 8, \"data\": [1,2,3,4], \"timestamp\": 0}";

	
    sendto(info->sniffer_fd, (const char *)hello, strlen(hello),
        0, (const struct sockaddr *) &servaddr,
            sizeof(servaddr));

	struct can_frame frame;
	char buffer[MAXLINE]; 
	while(1){
		nbytes = recvfrom(info->sniffer_fd, (char *)buffer, MAXLINE,
					MSG_WAITALL, (struct sockaddr *) &servaddr,
					(socklen_t*)&len);
					
		CANdata msg = {0};

		json_value *jv = json_parse(buffer, nbytes);
		json_object_entry *message = jv->u.object.values;
		msg.sid = message[0].value->u.integer; 
		msg.dlc = message[1].value->u.integer & 0xF;

		for (int i=0; i<4; i++) {
			msg.data[i] = message[2].value->u.array.values[i]->u.integer;
		}

		int dev_id = GET_DEV_ID(msg.sid);
		int msg_id = GET_MSG_ID(msg.sid);
		//printf("received from vsniffer: %d %d %d %x\n", dev_id, msg_id, msg.dlc, msg.data[0]);
		CANdata_to_CANFrame(msg, &frame);
		int bytesnum = send_to_CAN_Bus_Line(info->can_fd, frame);
		//printf("%d\n", bytesnum);
	}
}

void* vcan_to_vsniffer(void* arg){

	struct can_frame frame;
	CANdata msg;
	can_line_info *info = (can_line_info *)arg;
	struct sockaddr_in servaddr;
	memset(&servaddr, 0, sizeof(servaddr));

    // Filling server information
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(info->PORT);
    servaddr.sin_addr.s_addr = INADDR_ANY;

	for(;;){
		memset(&frame, 0, sizeof(struct can_frame));
		receive_from_CAN_Bus_Line(info->can_fd, &frame);
		CANFrame_to_CANdata(frame, &msg);

		if (!frame.can_dlc && !frame.can_id) continue;
		
		int timestamp = 69;
		char buffer[MAXLINE] = {0};
		sprintf(buffer, "{\"sid\": %d, \"dlc\": %d, \"data\": [%d,%d,%d,%d],\"timestamp\": %d}", msg.sid, msg.dlc, msg.data[0], msg.data[1],
				msg.data[2], msg.data[3], timestamp);

		//printf("%s\n", buffer);
		sendto(info->sniffer_fd, (const char *)buffer, strlen(buffer),
			0, (const struct sockaddr *) &servaddr,
				sizeof(servaddr));
	}
	printf("exit send_to_client\n");
	return NULL;
}

int send_to_CAN_Bus_Line(int sockfd, struct can_frame frame){
	int bytes_sent;
	bytes_sent = write(sockfd, &frame, sizeof(struct can_frame));
	return bytes_sent;
}

int receive_from_CAN_Bus_Line(int sockfd, struct can_frame* frame){
	fd_set rfds;
	int bytes_read = 0;
	FD_ZERO(&rfds);
    FD_SET(sockfd,&rfds);
	struct timeval timeout;
	timeout.tv_sec = 0;
	timeout.tv_usec = 0;

	int ret = select(sockfd+1,&rfds, NULL, NULL, &timeout);
	//printf("ret: %d\n", ret);
	if(FD_ISSET(sockfd,&rfds)){
		bytes_read = read(sockfd, frame, sizeof(struct can_frame));
	}

	return bytes_read;
	
}

void CANFrame_to_CANdata(struct can_frame frame, CANdata* message){
	message->sid = frame.can_id;

	message->dlc = frame.can_dlc;

	message->data[0] = frame.data[0] + (frame.data[1] << 8);
	message->data[1] = frame.data[2] + (frame.data[3] << 8);
	message->data[2] = frame.data[4] + (frame.data[5] << 8);
	message->data[3] = frame.data[6] + (frame.data[7] << 8);

	return;
}

void CANdata_to_CANFrame(CANdata message, struct can_frame* frame){

	frame->can_id = message.sid;
	frame->can_dlc = message.dlc;

	frame->data[0] = (unsigned char) (message.data[0] & 0x00FF);
	frame->data[1] = (unsigned char) (message.data[0] >> 8);

	frame->data[2] = (unsigned char) (message.data[1] & 0x00FF);
	frame->data[3] = (unsigned char) (message.data[1] >> 8);
	frame->data[4] = (unsigned char) (message.data[2] & 0x00FF);
	frame->data[5] = (unsigned char) (message.data[2] >> 8);
	frame->data[6] = (unsigned char) (message.data[3] & 0x00FF);
	frame->data[7] = (unsigned char) (message.data[3] >> 8);

	return;
}