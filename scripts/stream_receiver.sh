#!/bin/bash
gst-launch-1.0 udpsrc port=1234 ! application/x-rtp, encoding-name=H264, payload=96 ! rtph264depay ! h264parse ! queue ! avdec_h264 ! videoconvert ! autovideosink
