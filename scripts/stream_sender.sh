#!/bin/bash
gst-launch-1.0 v4l2src device=/dev/video0 ! jpegdec ! mpph264enc ! h264parse ! rtph264pay ! udpsink host=192.168.0.69 port=1234
