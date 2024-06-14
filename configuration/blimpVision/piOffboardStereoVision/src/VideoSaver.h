#pragma once

#include <string>
#include <stdio.h>
#include <opencv2/opencv.hpp>

class VideoSaver{

    std::string filename;
    double fps;
    bool verboseMode;
    cv::VideoWriter outputVideo;

    public:
        VideoSaver(std::string filename, double fps, bool verboseMode){
            this->filename = filename;
            this->fps = fps;
            this->verboseMode = verboseMode;
        }

        void writeFrame(cv::Mat frame){
            if(!outputVideo.isOpened()){
                if(verboseMode) fprintf(stdout, "Trying to open video (%s).\n", filename.c_str());
                outputVideo.open(filename, cv::VideoWriter::fourcc('M','J','P','G'), fps, frame.size(), true);
            }

            if(outputVideo.isOpened()){
                if(!frame.empty()){
                    outputVideo.write(frame);
                } else{
                    if(verboseMode) fprintf(stdout, "Attempted to write empty frame.\n");
                }
            }
            
        }

        ~VideoSaver(){
            if(verboseMode) fprintf(stdout, "Closing video (%s).", filename.c_str());
            outputVideo.release();
        }
};