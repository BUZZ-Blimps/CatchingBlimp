#!/bin/bash

python3 ../6.Run_This_Shell_Script/Computation/Yolov5_DeepSort_Pytorch/yolov5/train.py --img 960 --batch 32 --epoch 100 --data blimpLearn.yaml --weights yolov5s.pt
