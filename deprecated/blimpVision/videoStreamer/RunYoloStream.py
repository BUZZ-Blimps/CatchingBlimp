import cv2
import time
import sys
import numpy as np

def load_classes():
    class_list = []
    with open("config_files/classes2.txt", "r") as f:
        class_list = [cname.strip() for cname in f.readlines()]
    return class_list

def build_model(is_cuda):
    net = cv2.dnn.readNet("config_files/best_V3_720p.onnx")
    if is_cuda:
        #print("Attempty to use CUDA")
        net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
        net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA_FP16)
    else:
        #print("Running on CPU")
        net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
    return net

def format_yolov5(frame):
    row, col, _ = frame.shape
    _max = max(col, row)
    result = np.zeros((_max, _max, 3), np.uint8)
    result[0:row, 0:col] = frame
    return result

def detect(image, net, INPUT_WIDTH, INPUT_HEIGHT):
    blob = cv2.dnn.blobFromImage(image, 1 / 255.0, (INPUT_WIDTH, INPUT_HEIGHT), swapRB=True, crop=False)
    net.setInput(blob)
    preds = net.forward()
    return preds

def wrap_detection(input_image, output_data, INPUT_WIDTH, INPUT_HEIGHT):
    class_ids = []
    confidences = []
    boxes = []

    rows = output_data.shape[0]

    image_width, image_height, _ = input_image.shape

    x_factor = image_width / INPUT_WIDTH
    y_factor = image_height / INPUT_HEIGHT

    for r in range(rows):
        row = output_data[r]
        confidence = row[4]
        if confidence >= 0.4:

            classes_scores = row[5:]
            _, _, _, max_indx = cv2.minMaxLoc(classes_scores)
            class_id = max_indx[1]
            if (classes_scores[class_id] > .25):
                confidences.append(confidence)

                class_ids.append(class_id)

                x, y, w, h = row[0].item(), row[1].item(), row[2].item(), row[3].item()
                left = int((x - 0.5 * w) * x_factor)
                top = int((y - 0.5 * h) * y_factor)
                width = int(w * x_factor)
                height = int(h * y_factor)
                box = np.array([left, top, width, height])
                boxes.append(box)

    indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.25, 0.45)

    result_class_ids = []
    result_confidences = []
    result_boxes = []

    for i in indexes:
        result_confidences.append(confidences[i])
        result_class_ids.append(class_ids[i])
        result_boxes.append(boxes[i])

    return result_class_ids, result_confidences, result_boxes

def runYoloOnImage(imagePath):
    imagePath = imagePath.decode()
    INPUT_WIDTH = 640
    INPUT_HEIGHT = 640
    #SCORE_THRESHOLD = 0.2
    #NMS_THRESHOLD = 0.4
    #CONFIDENCE_THRESHOLD = 0.4

    class_list = load_classes()

    colors = [(255, 255, 0), (0, 255, 0), (0, 255, 255), (255, 0, 0)]

    is_cuda = False
    net = build_model(is_cuda)

    frame = cv2.imread(imagePath)

    retObjects = ""

    if frame is None:
        print("Bad Image.")
    else:
        inputImage = format_yolov5(frame)
        outs = detect(inputImage, net, INPUT_WIDTH, INPUT_HEIGHT)

        class_ids, confidences, boxes = wrap_detection(inputImage, outs[0], INPUT_WIDTH, INPUT_HEIGHT)
        retObjects += str(len(class_ids)) + ";"
        for (classid, confidence, box) in zip(class_ids, confidences, boxes):
            color = colors[int(classid) % len(colors)]
            cv2.rectangle(frame, box, color, 2)
            cv2.rectangle(frame, (box[0], box[1] - 20), (box[0] + box[2], box[1]), color, -1)
            cv2.putText(frame, class_list[classid], (box[0], box[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 0, 0))
            # Print the center x,y coordinate of the detection
            #print("X:" + str(box[0] + box[2] / 2), "Y:" + str(box[1] + box[3] / 2), "ID: " + str(class_list[classid]))
            retObjects += str(class_list[classid]) + "," + str(box[0] + box[2] / 2) + "," + str(box[1] + box[3] / 2) + ";"
    return retObjects.encode()

