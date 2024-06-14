#include "CompVisNew.h"

void CompVisNew::init()
{
    // Init video capture
    cam.open(CAMERA_INDEX, CAP_V4L);
    if (!cam.isOpened()) {
		CV_Assert("Cam open failed");
	}

}