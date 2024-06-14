#!/bin/bash

file="../1.Record_Video/Edit_Camera_Index.txt"
CAMERA_INDEX=$(cat "$file")
cd ../6.Run_This_Shell_Script/Computation/Yolov5_DeepSort_Pytorch
python3 yoloStream.py --yolo_model ../../../7.ML_Model_Output/ml_model/weights/best.pt --show-vid --img 960
echo ""
echo "Program Exited..."
echo ""
