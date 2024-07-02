import cv2
import time
from rknnpool import rknnPoolExecutor

from func import myFunc

cap = cv2.VideoCapture(0)
width = 1280
height = 640
cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
# cap = cv2.Vid

#modelPath = "./rknnModel/yolov5s_relu_tk2_RK3588_i8.rknn"
modelPath = "./rknnModel/blimps_v2.rknn"
# TPEs = 3
TPEs = 6

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
    half_frame = frame[:,0:640]
    cv2.imshow('test', half_frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    if frames % 30 == 0:
        print("30 frames average - frame rate:\t", 30 / (time.time() - loopTime), "frame")
        loopTime = time.time()

print("Overall Average Frame Rate\t", frames / (time.time() - initTime))

cap.release()
cv2.destroyAllWindows()
pool.release()
