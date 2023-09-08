# Machine Learning Pipeline

### Follow the steps below to create a new Machine Learning Model

1. Edit the Edit_Camera_Index.txt file in Folder 1 with the correct camera index you are using. Then run Record_Video.sh.

2. Run Divide_Video.sh in Folder 2.

3. Run Go_To_Label_Studio.sh in Folder 3. Choose images to label in the Images Folder in Folder 3. For more information read the Folder 3 README.md file. Export the Label Studio file as a Yolo zip file when done.

4. Place the Yolo zip file in Folder 4.

5. Edit the files in folder 5 if necesary.

6. Run ML_Pipeline.sh in Folder 6.

7. Machine learning model output will appear in Folder 7 as a folder titled ml_model. Run relocate_model.sh.

8. Run Run_ML_Tracking.sh to test the machine learning model on Steve.

9. Run Run_Pi_ML_Stream.sh to run the machine learning model on the Pi. (UNTESTED)
