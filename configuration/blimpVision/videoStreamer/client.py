#!/usr/bin/env python

from __future__ import division
import cv2
import numpy as np
import socket
import struct
import math
import json

MAX_DGRAM = 2**16
serverAddr = "192.168.0.220"
serverPort = 12345

recPort = 12344

class FrameSegment(object):
    """ 
    Object to break down image frame segment
    if the size of image exceed maximum datagram size 
    """
    MAX_DGRAM = 2**16
    MAX_IMAGE_DGRAM = MAX_DGRAM - 64 # extract 64 bytes in case UDP frame overflown
    def __init__(self, sock, port, addr=serverAddr):
        self.s = sock
        self.port = port
        self.addr = addr

    def udp_frame(self, img):
        """ 
        Compress image and Break down
        into data segments 
        """
        compress_img = cv2.imencode('.jpg', img)[1]
        dat = compress_img.tobytes()
        size = len(dat)
        count = math.ceil(size/(self.MAX_IMAGE_DGRAM))
        array_pos_start = 0
        while count:
            array_pos_end = min(size, array_pos_start + self.MAX_IMAGE_DGRAM)
            try:
                self.s.sendto(struct.pack("B", count) +
                    dat[array_pos_start:array_pos_end], 
                    (self.addr, self.port)
                    )
                array_pos_start = array_pos_end
                count -= 1
            except socket.error:
                continue

def main():
    """ Top level main function """
    # Set up UDP socket
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s_rec = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    s.setblocking(0)

    dat = b''

    fs = FrameSegment(s, serverPort)

    cap = cv2.VideoCapture(2, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    
    #cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    #cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    while (cap.isOpened()):
        _, frame = cap.read()
        fs.udp_frame(frame)

        try:
            seg, addr = s.recvfrom(MAX_DGRAM)
            if struct.unpack("B", seg[0:1])[0] > 1:
                dat += seg[1:]
            else:
                dat += seg[1:]
                jsonStr = dat.decode('utf-8')
                coords = json.loads(jsonStr)

                print('Received:')
                for key in coords:
                    print('{}: {}'.format(key, coords[key]))
                print('\n')

                dat = b''

        except socket.error:
            #Ignore timeout
            continue

    cap.release()
    cv2.destroyAllWindows()
    s.close()

if __name__ == "__main__":
    main()
