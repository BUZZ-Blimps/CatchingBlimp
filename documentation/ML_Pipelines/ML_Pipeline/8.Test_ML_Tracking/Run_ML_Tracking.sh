#!/bin/bash

file="../1.Record_Video/Edit_Camera_Index.txt"
CAMERA_INDEX=$(cat "$file")
cd ../6.Run_This_Shell_Script/Computation/Yolov5_DeepSort_Pytorch
python3 track.py --source $CAMERA_INDEX --yolo_model ../../../7.ML_Model_Output/ml_model/weights/best.pt --device cuda:0 --show-vid
echo ""
echo "Program Exited..."
echo ""
