#!/usr/bin/env python

##########################
## Description: Send (x, y) coordinates and objetc type to ros2 topic 'object_detection'.
## Note: 0 for green game ball | 1 for purple game ball | 2 for red blimp
##########################

import cv2
import time
from rknnpool import rknnPoolExecutor
from sort import Sort
from func import yolov5_post_process
import numpy as np
from geometry_msgs.msg import Point
import rclpy


IMG_HEIGHT, IMG_WIDTH = 480, 640
sort = Sort(2, 3, 0.3)

def get_max_area_type(boxes, scores, classes):
    max_area = 0
    max_type = None
    for box, score, cl in zip(boxes, scores, classes):
        x1, y1, x2, y2 = box
        # print(f'x1={x1}, y1={y1}, x2={x2}, y2={y2}')
        area = np.abs(x2-x1)*np.abs(y2-y1)
        if area > max_area:
            max_area = area
            max_type = cl
        
    return max_type, max_area

def get_distance(area, type):
    if type == 0 or type == 1:
        # distance = -3.955 * (10 ** -6) * area + 0.8219
        distance = 2.329*np.exp(-0.0001485*area) + 0.8616*np.exp(-8.479e-6*area)
        return distance
    else:
        return -1.0



def myFunc(rknn_lite, IMG):
    initial_IMG = IMG

    IMG = cv2.cvtColor(IMG, cv2.COLOR_BGR2RGB)
    IMG = IMG[:,0:IMG_WIDTH]
    IMG = cv2.resize(IMG, (IMG_WIDTH, IMG_HEIGHT))
    top_padding = (IMG_WIDTH - IMG_HEIGHT) // 2
    bottom_padding = (IMG_WIDTH - IMG_HEIGHT) - top_padding
    padded_frame = cv2.copyMakeBorder(IMG , top_padding, bottom_padding, 0, 0, cv2.BORDER_CONSTANT, value=[0, 0, 0])
    IMG = padded_frame

    initial_IMG = cv2.copyMakeBorder(initial_IMG , top_padding, bottom_padding, 0, 0, cv2.BORDER_CONSTANT, value=[0, 0, 0])

    outputs = rknn_lite.inference(inputs=[IMG])
    input0_data = outputs[0].reshape([3, -1]+list(outputs[0].shape[-2:]))
    input1_data = outputs[1].reshape([3, -1]+list(outputs[1].shape[-2:]))
    input2_data = outputs[2].reshape([3, -1]+list(outputs[2].shape[-2:]))
    input_data = list()
    input_data.append(np.transpose(input0_data, (2, 3, 0, 1)))
    input_data.append(np.transpose(input1_data, (2, 3, 0, 1)))
    input_data.append(np.transpose(input2_data, (2, 3, 0, 1)))

    boxes, classes, scores = yolov5_post_process(input_data)
    IMG = cv2.cvtColor(IMG, cv2.COLOR_RGB2BGR)
    max_coordinates_x = -1.0
    max_coordinates_y = -1.0
    max_type = -1.0
    z = -1.0

    max_coordinates_z = z

    dist = -1.0

    if boxes is not None:
        scores = scores.reshape((-1, 1))
        dets = np.concatenate((boxes, scores), 1)
        trackers = sort.update(dets)

        max_area = 0
        for trk in trackers:
            trk = trk.astype(int)
            cv2.rectangle(initial_IMG, (trk[0], trk[1]), (trk[2], trk[3]), (255, 0, 0), 3)
            cv2.putText(initial_IMG, str(trk[4]), (trk[0], trk[1]+12), 1, 1, (255, 255, 255))
            area = np.abs(trk[3]-trk[1])*np.abs(trk[2]-trk[0])
            if area > max_area:
                max_area = area
                max_coordinates_x = (trk[0]+trk[2])/2
                max_coordinates_y = (trk[1]+trk[3])/2
        
        max_type, area = get_max_area_type(boxes, scores, classes)
        # print(f'area = {area}')
        dist = get_distance(area, max_type)
        print(f'distance = {dist}')
        max_type = float(max_type)
    
    return initial_IMG, max_coordinates_x, max_coordinates_y, max_type, dist


cap = cv2.VideoCapture(0)
width = 1280
height = 480
cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

modelPath = "test_v5_640p.rknn"

TPEs = 3

pool = rknnPoolExecutor(
    rknnModel=modelPath,
    TPEs=TPEs,
    func=myFunc)

rclpy.init()
node = rclpy.create_node('opencv_publisher')
publisher = node.create_publisher(Point, 'object_detection', 10)
publisher2 = node.create_publisher(Point, 'distance', 10)

if (cap.isOpened()):
    for i in range(TPEs + 1):
        ret, frame = cap.read()
        if not ret:
            cap.release()
            del pool
            exit(-1)
        pool.put(frame)

while (cap.isOpened()):
    ret, frame = cap.read()
    if not ret:
        break
    pool.put(frame)
    detect_msg, flag = pool.get()

    msg = Point()
    msg.x = detect_msg[1]
    msg.y = detect_msg[2]
    msg.z = detect_msg[4]
    # msg.z = detect_msg[4]
    publisher.publish(msg)

    msg2 = Point()
    msg2.x = detect_msg[4]
    publisher2.publish(msg2)


    # msg = blimp_msg()
    # msg.x = detect_msg[1]
    # msg.y = detect_msg[2]
    # msg.z = detect_msg[3]
    # msg.type = detect_msg[4]
    # publisher.publish(msg)
    
    if flag == False:
        break
    detect_img = detect_msg[0]
    # cv2.imshow('test', detect_img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()
pool.release()
# rclpy.shutdown()