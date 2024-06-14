#include <iostream>
#include <iterator>
#include <algorithm>
#include <cmath>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>

#include <arpa/inet.h>
#include <fcntl.h>
#include <math.h>
#include <netinet/in.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#include "json.hpp"

using json = nlohmann::json;

#define MAXLINE 1024

const int MAX_DGRAM = pow(2, 16);
const int MAX_IMAGE_DGRAM = MAX_DGRAM - 64;

//const char* server_ip = "192.168.0.220";

const char* stream_server_ip = "127.0.0.1";
const int   stream_server_port = 12345;

int stream_socket_fd;
struct sockaddr_in stream_server_addr;

pthread_t stream_thread, stream_fb_thread;
long stream_thread_id, stream_fb_thread_id;

bool program_running = true;

void stop_program(int signal) {
	fprintf(stdout, "\nInterrupt received - exiting program.\n");
	program_running = false;

	//Wait for running threads to exit
	pthread_join(stream_thread, NULL);
	pthread_join(stream_fb_thread, NULL);
//	pthread_cancel(stream_thread);
//	pthread_cancel(stream_fb_thread);

	//Close the client socket if open
	if (stream_socket_fd) {
		close(stream_socket_fd);
	}

	exit(EXIT_SUCCESS);
}

void send_frame(cv::Mat image) {
    //Compress image into vector buffer
    std::vector<uchar> buf;
    cv::imencode(".jpg", image, buf);

    unsigned int size = (unsigned int)buf.size();
    unsigned int packet_count = ceil((double)size/(double)MAX_IMAGE_DGRAM);

    unsigned int array_pos_start = 0;
    while (packet_count > 0) {

    	//Grab the next subvector to populate the current data packet
    	int array_pos_end = std::min(size, array_pos_start + MAX_IMAGE_DGRAM);
    	std::vector<uchar>::const_iterator first = buf.begin() + array_pos_start;
    	std::vector<uchar>::const_iterator last = buf.begin() + array_pos_end;
    	std::vector<uchar> packet_buf(first, last);

    	//Convert the subvector buffer to an unsigned char array
    	unsigned int send_buf_len = packet_buf.size() + 1;
    	unsigned char send_buf[send_buf_len];
    	send_buf[0] = (unsigned char)packet_count; //First element is always the packet number
    	for (int i = 0; i < (int)packet_buf.size(); i++) {
    		send_buf[i+1] = packet_buf[i];
    	}

        sendto(stream_socket_fd, (const unsigned char *)send_buf, send_buf_len, MSG_CONFIRM, (const struct sockaddr *) &stream_server_addr, sizeof(stream_server_addr));

        array_pos_start = array_pos_end;
        packet_count--;
    }
}

void *stream_video(void *thread_id) {
	int t_id = (int)pthread_self();
	fprintf(stdout, "Video streaming thread (%d) successfully started.\n", t_id);

	cv:: Mat image;

    //Set up video capture
	cv::VideoCapture cap;
	int deviceID = 2;
	int apiID = cv::CAP_ANY;
    cap.open(deviceID, apiID);
    if (!cap.isOpened()) {
        std::cerr << "ERROR! Unable to open camera\n";
        return NULL;
    }
	cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);

	while (program_running) {
		//Read image off stereo cam
		if (cap.grab()) {
			cap.retrieve(image);

		    //Crop the image and stream it
		    cv::Rect left_roi(0, 0, image.cols/2, image.rows);
			cv::Mat imgL(image, left_roi);
		    send_frame(imgL);
		}
	}

	fprintf(stdout, "Video streaming thread (%d) successfully stopped.\n", t_id);
	pthread_exit((void *) 0);
	return NULL;
}

void *stream_fb_check(void *thread_id) {
	int t_id = (int)pthread_self();
	fprintf(stdout, "Stream feedback thread (%d) successfully started.\n", t_id);

	char buffer[MAXLINE];
	char sanitized_buffer[MAXLINE];
	unsigned int len = sizeof(stream_server_addr);
	while (program_running) {
		int n = recvfrom(stream_socket_fd, (char *)buffer, MAXLINE, MSG_WAITALL, (struct sockaddr *) &stream_server_addr, &len);
		if (n > 0) {
			int m = 0;
			for (int i = 0; i < n; i++) {
				//Sanitize the JSON buffer
				if (buffer[i] != '\1' && buffer[i] != '\10') {
					sanitized_buffer[m++] = buffer[i];
				}
			}
			sanitized_buffer[m] = '\0';

			json ex1 = json::parse(sanitized_buffer);

			std::cout << "x: " << ex1["x"] << ", y: " << ex1["y"] << std::endl;
		}
	}
	fprintf(stdout, "Stream feedback thread (%d) successfully stopped.\n", t_id);
	pthread_exit((void *) 0);
	return NULL;
}

void init_udp_streamer() {
    //Create UDP socket
	stream_socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (stream_socket_fd < 0) {
        perror("socket");
        exit(EXIT_FAILURE);
    }

    //Configure socket to be non-blocking
    fcntl(stream_socket_fd, F_SETFL, O_NONBLOCK);

    //Configure socket
    memset(&stream_server_addr, 0, sizeof(stream_server_addr));
    stream_server_addr.sin_family = AF_INET;
    stream_server_addr.sin_port = htons(stream_server_port);

    in_addr addr;
    inet_aton(stream_server_ip, &addr);
    stream_server_addr.sin_addr.s_addr = (in_addr_t)addr.s_addr;
}

int main(int argc, char** argv) {
    //Assign interrupt signals
    signal(SIGINT, 	stop_program);
    signal(SIGABRT, stop_program);
    signal(SIGKILL, stop_program);
    signal(SIGTERM, stop_program);
    signal(SIGTSTP, stop_program);

    //Start feedback stream thread
    if (pthread_create(&stream_thread, NULL, stream_video, (void *)stream_thread_id) < 0) {
        perror("Stream pthread_create");
        exit(EXIT_FAILURE);
    }

    if (pthread_create(&stream_fb_thread, NULL, stream_fb_check, (void *)stream_fb_thread_id) < 0) {
        perror("Stream feedback pthread_create");
        exit(EXIT_FAILURE);
    }

    while (1) {
    	//loop
    	sleep(1);
    }

    close(stream_socket_fd);
    return 0;
}
