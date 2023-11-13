# limit the number of cpus used by high performance libraries
import os
os.environ["OMP_NUM_THREADS"] = "1"
os.environ["OPENBLAS_NUM_THREADS"] = "1"
os.environ["MKL_NUM_THREADS"] = "1"
os.environ["VECLIB_MAXIMUM_THREADS"] = "1"
os.environ["NUMEXPR_NUM_THREADS"] = "1"

import sys
sys.path.insert(0, './yolov5')

import argparse
import os
import platform
import shutil
import time
from pathlib import Path
import cv2
import torch
import torch.backends.cudnn as cudnn
import numpy as np

from track_ros2.models.experimental import attempt_load
from track_ros2.utils.downloads import attempt_download
from track_ros2.models.common import DetectMultiBackend
# from track_ros2.utils.datasets import LoadImages, LoadStreams
from track_ros2.utils.general import (LOGGER, check_img_size, non_max_suppression, scale_coords, 
                                  check_imshow, xyxy2xywh, increment_path)
from track_ros2.utils.torch_utils import select_device, time_sync
from track_ros2.utils.plots import Annotator, colors
from track_ros2.deep_sort.utils.parser import get_config
from track_ros2.deep_sort.deep_sort import DeepSort
from track_ros2.utils.augmentations import  letterbox
from cv_bridge import CvBridge


#ROS imports 

import rclpy
from rclpy.node import Node
from yolo_msgs.msg import BoundingBoxes, BoundingBox, ObjectCount
from std_msgs.msg import Header

from sensor_msgs.msg import Image

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # yolov5 deepsort root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

 

class track_ros(Node):
    def __init__(self):
        super().__init__('track_ros')

        self.pub_bbox = self.create_publisher(BoundingBox, 'bounding_box',10)
        self.rect_image_subcriber = self.create_subscription(Image,'left/image_raw', self.rect_callback,10)
        self.tracker_polling_rate = self.create_timer(0.033,self.tracker_callback) #publish rate changed to ~30 Hz 
        #Parameters
        # FILE = Path(__file__).resolve()
        # ROOT = FILE.parents[0]  # yolov5 deepsort root directory
        # if str(ROOT) not in sys.path:
        #     sys.path.append(str(ROOT))  # add ROOT to PATH
        # ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

        self.declare_parameter('yolo_model', str(ROOT) + '/weights/best_test_960p.pt') #name of the model (put it in the weights folder)
        self.declare_parameter('source','0')
        self.declare_parameter('deep_sort_model', str(ROOT) + '/config/osnet_ibn_x1_0_MSMT17')
        self.declare_parameter('config_deepsort', str(ROOT) +'/deep_sort/configs/deep_sort.yaml')
        self.declare_parameter('imgsz', [1280,1280])
        self.declare_parameter('conf_thres', 0.5)
        self.declare_parameter('iou_thres', 0.5)
        self.declare_parameter('max_det', 1000)
        self.declare_parameter('device', 'cuda:0')
        self.declare_parameter('show_vid', False)
        self.declare_parameter('classes', None)
        self.declare_parameter('agnostic_nms', False)
        self.declare_parameter('half', False)
        self.declare_parameter('dnn', False)
        self.declare_parameter('augment', False)
        self.declare_parameter('visualize',False)



        self.yolo_model = self.get_parameter('yolo_model').value
        self.source = self.get_parameter('source').value
        self.deep_sort_model = self.get_parameter('deep_sort_model').value
        self.config_deepsort = self.get_parameter('config_deepsort').value
        self.imgsz = self.get_parameter('imgsz').value
        self.conf_thres = self.get_parameter('conf_thres').value
        self.iou_thres = self.get_parameter('iou_thres').value
        self.max_det = self.get_parameter('max_det').value
        self.device = self.get_parameter('device').value
        self.show_vid = self.get_parameter('show_vid').value
        self.classes = self.get_parameter('classes').value
        self.agnostic_nms = self.get_parameter('agnostic_nms').value
        self.half = self.get_parameter('half').value
        self.dnn = self.get_parameter('dnn').value
        self.augment = self.get_parameter('augment').value
        self.visualize = self.get_parameter('visualize').value

        #Might not be needed
        self.device = select_device(self.device)
        # initialize deepsort
        self.cfg = get_config()
        self.cfg.merge_from_file(self.config_deepsort)
        self.deepsort = DeepSort(self.deep_sort_model,
                            self.device,
                            max_dist=self.cfg.DEEPSORT.MAX_DIST,
                            max_iou_distance=self.cfg.DEEPSORT.MAX_IOU_DISTANCE,
                            max_age=self.cfg.DEEPSORT.MAX_AGE, n_init=self.cfg.DEEPSORT.N_INIT, nn_budget=self.cfg.DEEPSORT.NN_BUDGET,
                            )

        # Initialize
        self.half &= self.device.type != 'cpu'  # half precision only supported on CUDA

        self.br = CvBridge()
        self.current_frame = np.ones((1280, 1280, 3), dtype = np.uint8)
        self.current_time_sec = int(0)
        self.current_time_ns = int(0)
        self.current_frame_id = ''
        self.load_model()
        self.get_logger().info('Tracker Node Initialized')


    def tracker_callback(self):
       
        #Pad image for Yolo model 
        paddedImage = self.padRosImageForYolo()
        balloon_vals, y_goal_vals,o_goal_vals = self.detect_callback(paddedImage)
        
        headervals = Header()
        headervals.stamp.sec = self.current_time_sec
        headervals.stamp.nanosec = self.current_time_ns
        headervals.frame_id = self.current_frame_id
        msg = BoundingBox(header = headervals,x_center_balloon=balloon_vals["xCenter"],y_center_balloon=balloon_vals["yCenter"],width_balloon=balloon_vals["Width"],height_balloon=balloon_vals["Height"],
                            x_center_y_goal=y_goal_vals["xCenter"],y_center_y_goal=y_goal_vals["yCenter"],width_y_goal=y_goal_vals["Width"],height_y_goal=y_goal_vals["Height"],
                            x_center_o_goal=o_goal_vals["xCenter"],y_center_o_goal=o_goal_vals["yCenter"],width_o_goal=o_goal_vals["Width"],height_o_goal=o_goal_vals["Height"])
        self.pub_bbox.publish(msg)

         # msg_list = BoundingBoxes()
        ###TODO: Read in the UDP Stream here as an image and fix this to be in ros structure instead of infinite loop
        # for frame_idx, (path, image_raw, im0s, vid_cap, s) in enumerate(self.yolov5.dataset):
            # infoFromYolo = self.yolov5.detect_callback(image_raw,path,im0s,s)
        # image_raw= self.yolov5.dataset
               # if type(infoFromYolo) is list:
            #     for detection in infoFromYolo:
            #         # for key, value in d.items():
            #         #     print(key,value)
            #         try:
            #             if detection['xCenter'] is not None:
            #                     msg =BoundingBox(probability=float(detection['Confidence']),x_center=int(detection['xCenter']),
            #                     y_center=int(detection['yCenter']),width=int(detection['Width']),height=int(detection['Height']),
            #                     track_id =int(detection['ID']),class_id= int(detection['class']))
                                
            #                     msg_list.bounding_boxes.append(msg)
            #         except:
            #                 msg =BoundingBox(probability=float(detection['Confidence']),x_center=int(detection['xCenter']),
            #                     y_center=int(detection['yCenter']),width=int(detection['Width']),height=int(detection['Height']),
            #                     track_id =int(detection['ID']),class_id= int(detection['class']))
            #                 msg_list.bounding_boxes.append(msg)
            #               # print(key,value)
            #               # break
            #     self.pub_bboxes.publish(msg_list)
            #     print('Detections sent')
            # else:
            #     for key, value in infoFromYolo.items():
            #         # print(key,value)
            #         print('No detections sent')


    def rect_callback(self,data):
        self.current_frame = self.br.imgmsg_to_cv2(data)
        self.current_time_sec = data.header.stamp.sec
        self.current_time_ns = data.header.stamp.nanosec
        self.current_frame_id = data.header.frame_id

 
    def load_model(self):
        # Load model
        self.device = select_device(self.device)
        self.model = DetectMultiBackend(self.yolo_model, device=self.device, dnn=self.dnn)
        self.stride, names, self.pt, self.jit, _ = self.model.stride, self.model.names, self.model.pt, self.model.jit, self.model.onnx
        self.imgsz = check_img_size(self.imgsz, s=self.stride)  # check image size

        # Half
        self.half &= self.pt and self.device.type != 'cpu'  # half precision only supported by PyTorch on CUDA
        if self.pt:
            self.model.model.half() if self.half else self.model.model.float()

        #Process ROS2 Message Here
    def padRosImageForYolo(self):
        img = letterbox(self.current_frame, self.imgsz, stride=self.stride, auto=self.pt and not self.jit)[0]

        # Convert
        img = img.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
        img = np.ascontiguousarray(img)
        return img

    def detect_callback(self,img):
        #Initialize info to send
        #Check that the class indexes are correct
        balloon_vals = {"class":0,"xCenter":-1, "yCenter":-1,"Radius":-1,"Width":-1, "Height":-1, "Area": -1,"Confidence":-1}
        o_goal_vals = {"class":1,"xCenter":-1, "yCenter":-1,"Radius":-1, "Width":-1, "Height":-1,"Area": -1,"Confidence":-1}
        y_goal_vals = {"class":2,"xCenter":-1, "yCenter":-1,"Radius":-1, "Width":-1, "Height":-1,"Area": -1,"Confidence":-1}
       

        if self.pt and self.device.type != 'cpu':
            self.model(torch.zeros(1, 3, *self.imgsz).to(self.device).type_as(next(self.model.model.parameters())))  # warmup
        dt, seen = [0.0, 0.0, 0.0, 0.0], 0
        
        t1 = time_sync()
        img = torch.from_numpy(img).to(self.device)
        img = img.half() if self.half else img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0
        if img.ndimension() == 3:
            img = img.unsqueeze(0)
        t2 = time_sync()
        dt[0] += t2 - t1

        # Inference
        # visualize = increment_path(save_dir / Path(path[0]).stem, mkdir=True) if self.visualize else False
        pred = self.model(img, augment=self.augment, visualize=self.visualize)
        t3 = time_sync()
        dt[1] += t3 - t2

        # Apply NMS
        pred = non_max_suppression(pred, self.conf_thres, self.iou_thres, self.classes, self.agnostic_nms, max_det=self.max_det)
        dt[2] += time_sync() - t3

        # Process detections
        for i, det in enumerate(pred):  # detections per image
            seen += 1
            # if self.webcam:  # batch_size >= 1
            #     p, im0, _ = path[i], im0s[i].copy(), self.dataset.count
            #     s += f'{i}: '
            # else:
            #     p, im0, _ = path, im0s.copy(), getattr(self.dataset, 'frame', 0)

            # p = Path(p)  # to Path
            # save_path = str(save_dir / p.name)  # im.jpg, vid.mp4, ...
            # s += '%gx%g ' % img.shape[2:]  # print string

            # annotator = Annotator(im0, line_width=2, pil=not ascii)
            annotator = Annotator(img, line_width=2, pil=not ascii)

            if det is not None and len(det):
                # Rescale boxes from img_size to im0 size
                # det[:, :4] = scale_coords(
                #     img.shape[2:], det[:, :4], im0.shape).round()

                # Print results
                # for c in det[:, -1].unique():
                #     n = (det[:, -1] == c).sum()  # detections per class
                #     s += f"{n} {names[int(c)]}{'s' * (n > 1)}, "  # add to string

                xywhs = xyxy2xywh(det[:, 0:4])
                confs = det[:, 4]
                clss = det[:, 5]

                # pass detections to deepsort
                t4 = time_sync()
                # outputs = self.deepsort.update(xywhs.cpu(), confs.cpu(), clss.cpu(), im0)
                # print(type(img))
                # print(type(img.cpu()))
                # print(type(np.array(img.cpu())))
                # print(np.reshape(img.cpu(),(576,736,3)).shape)
                arrImage = np.reshape(img.cpu().numpy(),(960,1280,3)) #height, width (960,1280)
                correctedImage = cv2.cvtColor(arrImage, cv2.COLOR_RGB2BGR).astype(np.uint8)
                outputs = self.deepsort.update(xywhs.cpu(), confs.cpu(), clss.cpu(), correctedImage)
                t5 = time_sync()
                dt[3] += t5 - t4

                # draw boxes for visualization
                if len(outputs) > 0:
                    # infoToSendfromYolo = []
                    for j, (output, conf) in enumerate(zip(outputs, confs)):

                        bboxes = output[0:4]
                        id = output[4]
                        cls = output[5]

                        c = int(cls)  # integer class
                        # label = f'{id} {names[c]} {conf:.2f}'
                        # annotator.box_label(bboxes, label, color=colors(c, True))
                        #Send bounding boxes to PI, center of x,y,radius, area, confidence
                        width = (output[2] - output[0])
                        height = (output[3] - output[1])
                        xCenter = output[0]+width/2
                        yCenter = output[1]+height/2
                        radius = -1
                        area = output

                            #A more efficient way to find the largest detection is possible, but this will work for now.
                        info = {"class":int(c),"ID":int(id),"xCenter":int(xCenter), "yCenter":int(yCenter),"Radius":int(radius),"Width":int(width),"Height":int(height), "Area": int(width*height),"Confidence":int(conf.cpu().numpy())}
                        #Updated 9/22/23 as of settings in label studio
                        if c == 0 and balloon_vals["Area"]<= width*height:
                            balloon_vals = info
                       #TODO: Add in functionality for red and blue blimps when needed
                       # if c == 1 and blue_blimp_vals["Area"]<=width*height:
                          #  blue_blimp_vals = info
                        #if c == 3 and red_blimp_vals["Area"]<= width*height:
                           # red_blimp_vals = info
                        if c == 2 and o_goal_vals["Area"]<= width*height:
                            o_goal_vals = info
                        if c == 4 and y_goal_vals["Area"]<=width*height:
                            y_goal_vals = info

                    

                # LOGGER.info(f'{s}Done. YOLO:({t3 - t2:.3f}s), DeepSort:({t5 - t4:.3f}s)')

            else:
                self.deepsort.increment_ages()
                # LOGGER.info('No detections')

        # return infoToSendfromYolo
        return [balloon_vals, y_goal_vals, o_goal_vals]


def main(args=None):
    rclpy.init(args=args)
    track_node = track_ros()
    rclpy.spin(track_node)
    track_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
