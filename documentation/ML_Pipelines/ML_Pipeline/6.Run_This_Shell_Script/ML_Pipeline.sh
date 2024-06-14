#!/bin/bash

# Machine Learning Pipeline

# 1. Take a Video (Run piRecordVideo to get a .avi video file)
# *Make Sure Camera Index is Correct!*
# Edit the Camera Index File
# Run Record_Video.sh
# . Record_Video.sh
# or
# ./Record_Video.sh

# 2. Divide video into Images by running the command: ffmpeg -i [filename.avi] im%05d.png (combine with step 1a)
# Run Divide_Video.sh
# . Divide_Video.sh
# or
# ./Divide_Video.sh

# 3. Label Images (Must be done manually unless we train a separate machine learning model to do it autonomously. Currently using labelstud.io. Gives use the images and corresponding labels)
# Run Go_To_Label_Studio.sh and read the readme file to access Label Studio from any Computer
# Import new images from the Images Folder in Folder 2 and label them
# Export as a Yolo Zip File
# ./Go_To_Label_Studio.sh

# 4. Place the Yolo Zip File in Folder 4 with any name

# 5. Edit the files in Folder 5

# 6. Run ML_Pipeline.sh in Folder 6
# . ML_Pipeline.sh
# or
# ./ML_Pipeline.sh

# 6a. Get the augmented images with corresponding labels (Run alb_main.py with images and labels in the same folder)
cd ../4.Place_LS_Zip_Folder
unzip *.zip -d $PWD/yolo_gameball
mv yolo_gameball ../6.Run_This_Shell_Script/Computation/ml_dataset
cd ../6.Run_This_Shell_Script/Computation/ml_dataset
mkdir tempImages
mkdir tempLabels
cd yolo_gameball/images
mv *.png ../../albFolder
cd ../labels
mv *.txt ../../albFolder
cd ../../albFolder
python3 alb_main.py
mv *.png ../tempImages
mv *.txt ../tempLabels

# 6b. Move approximately 80% of the images and labels into their own folder in a train folder and the rest into a valid folder. Make a yaml file for the folder.
# Ex. yolo_gameball_test (main folder)
#	-> train
#		-> images
#		-> labels
#	-> valid
#		-> images
#		-> labels
cd ..
mkdir train
cd train
mkdir images
mkdir labels
cd ..

mkdir valid
cd valid
mkdir images
mkdir labels
cd ..

cd tempImages
TOTAL_IMAGES=$(ls -1 --file-type | grep -v '/$' | wc -l)
PERCENT=0.80
TEMP_NUM_IMAGES=$(echo "scale=4; $TOTAL_IMAGES*$PERCENT" | bc)
NUM_IMAGES=${TEMP_NUM_IMAGES%.*}
ls | head -n $NUM_IMAGES | xargs -I{} mv {} ../train/images
mv *.png ../valid/images

cd ../tempLabels
TOTAL_LABELS=$(ls -1 --file-type | grep -v '/$' | wc -l)
TEMP_NUM_LABELS=$(echo "scale=4; $TOTAL_LABELS*$PERCENT" | bc)
NUM_LABELS=${TEMP_NUM_LABELS%.*}
ls | head -n $NUM_LABELS | xargs -I{} mv {} ../train/labels
mv *.txt ../valid/labels

cd ..
rm -r yolo_gameball
rm -r albFolder
rm -r tempImages
rm -r tempLabels

# 6c. Add the main folder created in 3b to the datasets folder in Yolov5_DeepSort_Pytorch. Also add the yaml file to the data folder as well as to another data folder in the yolov5 folder.

# Create yaml file or already have it in the folder ml_dataset
cd ..
mv ml_dataset Yolov5_DeepSort_Pytorch/datasets
cp ../../5.Edit_These_Files/*.yaml Yolov5_DeepSort_Pytorch/data
cp ../../5.Edit_These_Files/*.yaml Yolov5_DeepSort_Pytorch/yolov5/data
cd ../../5.Edit_These_Files

# 6d. Run the Train_Dataset.sh file with the correct images resolution and yaml file. This creates a pt file that is used for streaming with the stereocamera.

# Add Parameters to the sh file below for image resolution, yaml file name, and pt file name
chmod +x Train_Dataset.sh
./Train_Dataset.sh
cd ..

# 7. Save the pt file and run track.py with a video or yoloStream.py to check your newly created machine learning model
cd 6.Run_This_Shell_Script/Computation/Yolov5_DeepSort_Pytorch/yolov5/runs/train
cp -r "$(ls -t -d */ | head -n1)" ../../../../../../7.ML_Model_Output
cd ../../../../../../..
FILENAME=ML_Pipeline_COMPLETE_$(date +%m-%d-%Y_%I:%M:%S%p)
mv ML_Pipeline Completed/$FILENAME
cd Completed/$FILENAME/7.ML_Model_Output
mv "$(ls -t -d */ | head -n1)" ml_model
cd "$(ls -t -d */ | head -n1)"
ls

echo ""
echo "ML Model Creation Successful"
echo ""
