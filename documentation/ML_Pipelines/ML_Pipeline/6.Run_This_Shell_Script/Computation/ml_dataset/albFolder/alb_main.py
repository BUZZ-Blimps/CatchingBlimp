from cProfile import label
import albumentations as A
import cv2
import os
import copy
from matplotlib import pyplot as plt
import glob

BOX_COLOR = (255, 0, 0) # Red
TEXT_COLOR = (255, 255, 255) # White


def visualize_bbox(img, bbox, class_name, color=BOX_COLOR, thickness=2):
    """Visualizes a single bounding box on the image"""
    x_min, y_min, w, h = bbox

    x_min, x_max, y_min, y_max = int((x_min-w/2)*IMG_WIDTH), int((x_min + w/2)*IMG_WIDTH), int((y_min-h/2)*IMG_HEIGHT), int((y_min + h/2)*IMG_HEIGHT)
   
    cv2.rectangle(img, (x_min, y_min), (x_max, y_max), color=color, thickness=thickness)
    
    ((text_width, text_height), _) = cv2.getTextSize(class_name, cv2.FONT_HERSHEY_SIMPLEX, 0.35, 1)    
    cv2.rectangle(img, (x_min, y_min - int(1.3 * text_height)), (x_min + text_width, y_min), BOX_COLOR, -1)
    cv2.putText(
        img,
        text=class_name,
        org=(x_min, y_min - int(0.3 * text_height)),
        fontFace=cv2.FONT_HERSHEY_SIMPLEX,
        fontScale=0.35, 
        color=TEXT_COLOR, 
        lineType=cv2.LINE_AA,
    )
    return img


def visualize(image, bboxes, class_label):
    img = image.copy()
    i=0
    for bbox in bboxes:
        class_name = class_label[i]


        img = visualize_bbox(img, bbox, class_name)
        i = i+1
    plt.figure(figsize=(12, 12))
    plt.axis('off')
    plt.imshow(img)
    print("Reached end of visualize method")
    # return bboxes

def readImage(filename):
    # OpenCV uses BGR channels
    img = cv2.imread(filename)
    return img

def readYolo(filename):
    coords = []
    with open(filename, 'r') as fname:
        for file1 in fname:
            x = file1.strip().split(' ')
            # x.append(x[0])
            x.pop(0)
            x[0] = float(x[0])
            x[1] = float(x[1])
            x[2] = float(x[2])
            x[3] = float(x[3])
            coords.append(x)
    return coords
def readLabelsYolo(filename):
    labels = []
    with open(filename, 'r') as fname:
        for file1 in fname:
            components = file1.split(" ")
            if(components[0]=='0'):
                category = "Balloon"
            elif(components[0]=='1'):
                category = "Blue Blimp"
            elif(components[0]=='2'):
                category = "Red Blimp"
            elif(components[0]=='3'):
                category = "Orange Goal"
            elif(components[0]=='4'):
                category = "Yellow Goal"
            labels.append(category)
 
    return labels

def writeYolo(coords,class_labels, count, name):
    i = 0
    labelWrite = -1
    with open(name+str(count)+'.txt', "w") as f:
        for x in coords:
            if class_labels[i]=='Balloon':
                labelWrite = 0
            elif class_labels[i] == 'Blue Blimp':
                labelWrite = 1
            elif class_labels[i] == 'Red Blimp':
                labelWrite = 2
            elif class_labels[i] == 'Orange Goal':
                labelWrite = 3
            elif class_labels[i] == 'Yellow Goal':
                labelWrite = 4
            # print("%s %s %s %s %s \n" % (labelWrite, x[0], x[1], x[2], x[3]))
            f.write("%s %s %s %s %s \n" % (labelWrite, x[0], x[1], x[2], x[3]))
            # f.write("%s %s %s %s %s \n" % (x[-1], x[0], x[1], x[2], x[3]))
            i = i + 1
transform = A.Compose([
    # A.RandomCrop(width=450, height=450),
    A.MotionBlur(p=0.5),
    # A.PiecewiseAffine(p=0.2),
    # A.RandomSizedBBoxSafeCrop(width=1920, height=1080,p=1.0),
    # A.RGBShift(r_shift_limit = 0,g_shift_limit = 0,b_shift_limit = 0,p=1.0),
    A.HorizontalFlip(p=0.5),
    A.RandomBrightnessContrast(p=0.3),
    # A.HueSaturationValue(p=1.0),
    A.PixelDropout(p=0.2),
    #A.ChannelDropout(p=0.2),
    #A.ChannelShuffle(p=0.2),
], bbox_params=A.BboxParams(format='yolo',label_fields=['class_labels']))



count = 3000
numberOfAugmentedImages = 10
textpath = './*.txt' 
imagepath = './*.png'

txtFiles = glob.glob(textpath)
imgFiles = glob.glob(imagepath)
# print(txtFiles)
# print(imgFiles)

for filenameImg in imgFiles:
    imgCurrentName, ext = os.path.splitext(os.path.basename(filenameImg))
    imgCurr = readImage(imgCurrentName+'.png')
    print(imgCurrentName)
    for txtFileName in txtFiles:
        txtFileName, ext = os.path.splitext(os.path.basename(txtFileName))
        print(txtFileName)
        if txtFileName == imgCurrentName:
            bboxes = readYolo(txtFileName+'.txt')
            class_label = readLabelsYolo(txtFileName+'.txt')
            print(bboxes)
            print(class_label)
            for i in range(0, numberOfAugmentedImages):
                img = copy.deepcopy(imgCurr)
                # transform = getTransform(i)

                try:
                    transformed = transform(image=img, bboxes=bboxes,class_labels= class_label)
                    transformed_image = transformed['image']
                    transformed_bboxes = transformed['bboxes']
                    transformed_class_labels = transformed['class_labels']
                    name = imgCurrentName+str(count)+'.png'
                    
                    cv2.imwrite(name, transformed_image)
                    # print(transformed_bboxes)
                    # writeVoc(transformed_bboxes, count, transformed_image)
                    writeYolo(transformed_bboxes,transformed_class_labels, count, imgCurrentName)
                    count = count+1
                except:
                    print("bounding box issues")
                    pass



