#!/usr/bin/env python

from __future__ import division
import cv2
import numpy as np
import socket
import struct
import math
import time
import json

import sys

MAX_DGRAM = 2**16

serverAddr = "192.168.0.202"
#serverAddr = "127.0.0.1"

serverPort = 12345

def dump_buffer(s):
    """ Emptying buffer frame """
    s.setblocking(0)
    while True:
        try:
            seg, addr = s.recvfrom(MAX_DGRAM)
            if struct.unpack("B", seg[0:1])[0] == 1:
                break
        except socket.error:
            #Nothing to receive
            break
    s.setblocking(1)

def send_ack(s, addr, port):
    coords = {
        "type": 0,
        "x": np.random.normal(0, 1.0),
        "y": np.random.normal(0, 1.0),
        "z": np.random.normal(0, 1.0),
        "Luke": "hello world",
        "egg_in_cup?": True
    }

    data = bytes(json.dumps(coords), 'utf-8')
    size = len(data)
    count = math.ceil(size/(MAX_DGRAM))
    array_pos_start = 0
    while count:
        array_pos_end = min(size, array_pos_start + MAX_DGRAM)
        s.sendto(struct.pack("B", count) +
            data[array_pos_start:array_pos_end],
            (addr, port)
            )
        array_pos_start = array_pos_end
        count -= 1

def main():
    """ Getting image udp frame &
    concate before decode and output image """
    # Set up socket
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind((serverAddr, serverPort))
    dat = b''
    dump_buffer(s)

    while True:
        seg, addr = s.recvfrom(MAX_DGRAM)
        packet_count = struct.unpack("B", seg[0:1])[0]
        if packet_count > 1:
            dat += seg[1:]
        else:
            dat += seg[1:]

            img = cv2.imdecode(np.frombuffer(dat, dtype=np.uint8), cv2.IMREAD_UNCHANGED)

            if (img is not None):
                cv2.imshow('Video Stream', img)
                send_ack(s, addr[0], addr[1])

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            dat = b''

    # cap.release()
    cv2.destroyAllWindows()
    s.close()

if __name__ == "__main__":
    main()

