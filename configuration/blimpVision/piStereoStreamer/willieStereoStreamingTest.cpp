//============================================================================
// Name        : willieStereoTest.cpp
// Author      : Willie
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

//C++ includes
#include <algorithm>
#include <cmath>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <iostream>
#include <iterator>
#include <list>
#include <unordered_map>
#include <string>
#include <vector>

//C includes
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <libv4l2.h>
#include <linux/videodev2.h>
#include <math.h>
#include <netinet/in.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

//OpenCV includes
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/ximgproc/disparity_filter.hpp>

#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/ximgproc.hpp"

#include "piOffboardStereoVision.hpp"

using namespace std;

void stop_program(int signal) {
	fprintf(stdout, "\nInterrupt received - exiting program.\n");

	//Send all threads a termination flag
	program_running = false;

	//Exit successfully
	exit(EXIT_SUCCESS);
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

//Helper function for video streaming thread
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
    	send_buf[0] = (unsigned char)packet_count; //First element is always the # of remaining packets
    	for (int i = 0; i < (int)packet_buf.size(); i++) {
    		send_buf[i+1] = packet_buf[i];
    	}

        sendto(stream_socket_fd, (const unsigned char *)send_buf, send_buf_len, MSG_CONFIRM, (const struct sockaddr *) &stream_server_addr, sizeof(stream_server_addr));

        array_pos_start = array_pos_end;
        packet_count--;
    }
}

int main() {

	cout << "opencv version: " << CV_VERSION << endl;

    //Assign interrupt signals
    signal(SIGINT, 	stop_program);
    signal(SIGABRT, stop_program);
    signal(SIGKILL, stop_program);
    signal(SIGTERM, stop_program);
    signal(SIGTSTP, stop_program);

    init_udp_streamer();

	cv:: Mat image;

    //Set up video capture
	cv::VideoCapture cap;

    //Make sure camera is connected
//    if (!cap.isOpened()) {
//		CV_Assert("Cam open failed");
//		cout << "camera failed to open" << endl;
//		return 0;
//    }

	int cap_api_id = cv::CAP_ANY;
    cap.open(0, cap_api_id);
    if (!cap.isOpened()) {
        std::cerr << "ERROR! Unable to open camera\n";
        return -1;
    }

    //Set the stereo cam to full resolution
	cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);

	while (program_running) {

		//Read image off stereo cam
		if (cap.grab()) {
			cap.retrieve(image);

		    //Crop the left and right images
		    cv::Rect left_roi(0, 0, image.cols/2, image.rows);
		    cv::Rect right_roi(image.cols/2, 0, image.cols/2, image.rows);
			cv::Mat lt_frame_maxres(image, left_roi);
			cv::Mat rt_frame_maxres(image, right_roi);

			//Reduce image size for rectification
			cv::resize(lt_frame_maxres, lt_frame_lowres, cv::Size(RECT_WIDTH, RECT_HEIGHT), cv::INTER_LINEAR);
			cv::resize(rt_frame_maxres, rt_frame_lowres, cv::Size(RECT_WIDTH, RECT_HEIGHT), cv::INTER_LINEAR);

			//Stream left frame to offboard server
			send_frame(lt_frame_maxres);

			//Uncomment to display transmitted video
//			imshow("Left Camera", lt_frame_maxres);
//			cv::waitKey(1);
		}

//		if (annotatedMode && annotatedFrameReady){
//			annotatedFrameReady = false;
//			cv::Mat annotatedFrameToSend;
//			pthread_mutex_lock(&annotatedFrameMutex);
//			annotatedFrame.copyTo(annotatedFrameToSend);
//			pthread_mutex_unlock(&annotatedFrameMutex);
//			send_frame(annotatedFrameToSend);
//		}
	}

    cap.release();

	return 0;
}
