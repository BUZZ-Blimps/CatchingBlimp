import cv2
import time
from rknnpool import rknnPoolExecutor
from sort import Sort
from func import yolov5_post_process
import numpy as np

IMG_HEIGHT, IMG_WIDTH = 480, 640

# sort = Sort(2, 3, 0.3)

def myFunc(rknn_lite, IMG):
    sort = Sort(2, 3, 0.3)
    initial_IMG = IMG

    IMG = cv2.cvtColor(IMG, cv2.COLOR_BGR2RGB)
    IMG = IMG[:,0:IMG_WIDTH]
    IMG = cv2.resize(IMG, (IMG_WIDTH, IMG_HEIGHT))
    top_padding = (IMG_WIDTH - IMG_HEIGHT) // 2
    bottom_padding = (IMG_WIDTH - IMG_HEIGHT) - top_padding
    padded_frame = cv2.copyMakeBorder(IMG , top_padding, bottom_padding, 0, 0, cv2.BORDER_CONSTANT, value=[0, 0, 0])
    IMG = padded_frame

    initial_IMG = cv2.copyMakeBorder(initial_IMG , top_padding, bottom_padding, 0, 0, cv2.BORDER_CONSTANT, value=[0, 0, 0])
    # IMG = np.expand_dims(IMG, axis=0)
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
    if boxes is not None:
        # draw(IMG, boxes, scores, classes)
        scores = scores.reshape((-1, 1))
        dets = np.concatenate((boxes, scores), 1)
        trackers = sort.update(dets)
        for trk in trackers:
            trk = trk.astype(int)
            # cv2.rectangle(IMG, (trk[0], trk[1]), (trk[2], trk[3]), (255, 0, 0), 3)
            cv2.rectangle(initial_IMG, (trk[0], trk[1]), (trk[2], trk[3]), (255, 0, 0), 3)
            # cv2.putText(IMG, str(trk[4]), (trk[0], trk[1]+12), 1, 1, (255, 255, 255))
    return initial_IMG[:, :640]




cap = cv2.VideoCapture(0)
width = 1280
height = 480
cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

modelPath = "test_v5_640p.rknn"

TPEs = 3
# TPEs = 6

pool = rknnPoolExecutor(
    rknnModel=modelPath,
    TPEs=TPEs,
    func=myFunc)



if (cap.isOpened()):
    for i in range(TPEs + 1):
        ret, frame = cap.read()
        if not ret:
            cap.release()
            del pool
            exit(-1)
        pool.put(frame)

frames, loopTime, initTime = 0, time.time(), time.time()
while (cap.isOpened()):
    frames += 1
    ret, frame = cap.read()
    if not ret:
        break
    pool.put(frame)
    frame, flag = pool.get()
    if flag == False:
        break

    half_frame = frame
   
    cv2.imshow('test', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    if frames % 30 == 0:
        print("30 frames average - frame rate:\t", 30 / (time.time() - loopTime), "frame")
        loopTime = time.time()

print("Overall Average Frame Rate\t", frames / (time.time() - initTime))

cap.release()
cv2.destroyAllWindows()
pool.release()
