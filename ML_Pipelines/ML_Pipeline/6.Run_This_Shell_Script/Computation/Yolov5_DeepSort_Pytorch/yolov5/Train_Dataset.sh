#!/bin/bash

python3 Yolov5_DeepSort_Pytorch/train.py --img 720 --batch 32 --epoch 100 --data blimpLearn.yaml --weights yolov5s.pt
