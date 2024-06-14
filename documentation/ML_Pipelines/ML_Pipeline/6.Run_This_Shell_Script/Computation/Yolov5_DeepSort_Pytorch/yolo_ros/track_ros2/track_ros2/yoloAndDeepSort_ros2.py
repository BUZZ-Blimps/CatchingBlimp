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

from yolov5.models.experimental import attempt_load
from yolov5.utils.downloads import attempt_download
from yolov5.models.common import DetectMultiBackend
from yolov5.utils.datasets import LoadImages, LoadStreams
from yolov5.utils.general import (LOGGER, check_img_size, non_max_suppression, scale_coords, 
                                  check_imshow, xyxy2xywh, increment_path)
from yolov5.utils.torch_utils import select_device, time_sync
from yolov5.utils.plots import Annotator, colors
from deep_sort.utils.parser import get_config
from deep_sort.deep_sort import DeepSort


#ROS imports 

import rclpy
from rclpy.node import Node
from yolo_ros_msgs.msg import BoundingBoxes, BoundingBox, ObjectCount
from std_msgs.msg import Header


FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # yolov5 deepsort root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

class track():
    def __init__(self, source, yolo_model, deep_sort_model,config_deepsort,visualize,augment show_vid, imgsz,half,dnn,device, max_det,conf_thres,iou_thres,classes,agnostic_nms):
      
        self.source = source
        self.yolo_model = yolo_model
        self.deep_sort_model = deep_sort_model
        self.show_vid = show_vid
        self.imgsz = imgsz 
        self.half = half
        self.dnn = dnn
        self.device = device
        self.visualize = visualize 
        self.augment = augment
        self.config_deepsort = config_deepsort
        self.max_det = max_det
        self.conf_thres = conf_thres
        self.iou_thres = iou_thres
        self.classes = classes
        self.agnostic_nms = agnostic_nms

        webcam = self.source == '0' or self.source.startswith(
            'rtsp') or self.source.startswith('http') or self.source.endswith('.txt') 
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
     
        self.load_model()

    def load_model(self):
        # Load model
        self.device = select_device(self.device)
        self.model = DetectMultiBackend(self.yolo_model, device=self.device, dnn=self.dnn)
        stride, names, self.pt, jit, _ = self.model.stride, self.model.names, self.model.pt, self.model.jit, self.model.onnx
        self.imgsz = check_img_size(self.imgsz, s=stride)  # check image size

        # Half
        self.half &= self.pt and self.device.type != 'cpu'  # half precision only supported by PyTorch on CUDA
        if pt:
            self.model.model.half() if half else self.model.model.float()

        # Set Dataloader
        vid_path, vid_writer = None, None
        # Check if environment supports image displays
        if show_vid:
            show_vid = check_imshow()

        # Dataloader
        if webcam:
            show_vid = check_imshow()
            cudnn.benchmark = True  # set True to speed up constant image size inference
            self.dataset = LoadStreams(self.source, img_size=self.imgsz, stride=stride, auto=self.pt and not jit)
            bs = len(self.dataset)  # batch_size
        else:
            self.dataset = LoadImages(self.source, img_size=self.imgsz, stride=stride, auto=self.pt and not jit)
            bs = 1  # batch_size
        vid_path, vid_writer = [None] * bs, [None] * bs

        # Get names and colors
        names = model.module.names if hasattr(model, 'module') else model.names

        # extract what is in between the last '/' and last '.'
        txt_file_name = source.split('/')[-1].split('.')[0]
        txt_path = str(Path(save_dir)) + '/' + txt_file_name + '.txt'



    def detect_callback(self,img):
        if self.pt and self.device.type != 'cpu':
            model(torch.zeros(1, 3, *self.imgsz).to(self.device).type_as(next(self.model.model.parameters())))  # warmup
        dt, seen = [0.0, 0.0, 0.0, 0.0], 0
        for frame_idx, (path, img, im0s, vid_cap, s) in enumerate(self.dataset):
            t1 = time_sync()
            img = torch.from_numpy(img).to(self.device)
            img = img.half() if half else img.float()  # uint8 to fp16/32
            img /= 255.0  # 0 - 255 to 0.0 - 1.0
            if img.ndimension() == 3:
                img = img.unsqueeze(0)
            t2 = time_sync()
            dt[0] += t2 - t1

            # Inference
            visualize = increment_path(save_dir / Path(path[0]).stem, mkdir=True) if self.visualize else False
            pred = model(img, augment=self.augment, visualize=self.visualize)
            t3 = time_sync()
            dt[1] += t3 - t2

            # Apply NMS
            pred = non_max_suppression(pred, self.conf_thres, self.iou_thres, self.classes, self.agnostic_nms, max_det=self.max_det)
            dt[2] += time_sync() - t3

            # Process detections
            for i, det in enumerate(pred):  # detections per image
                seen += 1
                if webcam:  # batch_size >= 1
                    p, im0, _ = path[i], im0s[i].copy(), self.dataset.count
                    s += f'{i}: '
                else:
                    p, im0, _ = path, im0s.copy(), getattr(self.dataset, 'frame', 0)

                p = Path(p)  # to Path
                save_path = str(save_dir / p.name)  # im.jpg, vid.mp4, ...
                s += '%gx%g ' % img.shape[2:]  # print string

                annotator = Annotator(im0, line_width=2, pil=not ascii)

                if det is not None and len(det):
                    # Rescale boxes from img_size to im0 size
                    det[:, :4] = scale_coords(
                        img.shape[2:], det[:, :4], im0.shape).round()

                    # Print results
                    for c in det[:, -1].unique():
                        n = (det[:, -1] == c).sum()  # detections per class
                        s += f"{n} {names[int(c)]}{'s' * (n > 1)}, "  # add to string

                    xywhs = xyxy2xywh(det[:, 0:4])
                    confs = det[:, 4]
                    clss = det[:, 5]

                    # pass detections to deepsort
                    t4 = time_sync()
                    outputs = deepsort.update(xywhs.cpu(), confs.cpu(), clss.cpu(), im0)
                    t5 = time_sync()
                    dt[3] += t5 - t4

                    # draw boxes for visualization
                    if len(outputs) > 0:
                        infoToSendfromYolo = []
                        for j, (output, conf) in enumerate(zip(outputs, confs)):

                            bboxes = output[0:4]
                            id = output[4]
                            cls = output[5]

                            c = int(cls)  # integer class
                            label = f'{id} {names[c]} {conf:.2f}'
                            annotator.box_label(bboxes, label, color=colors(c, True))
                            #Send bounding boxes to PI, center of x,y,radius, area, confidence
                            width = (output[2] - output[0])
                            height = (output[3] - output[1])
                            xCenter = output[0]+width/2
                            yCenter = output[1]+height/2
                            radius = -1
                            area = output

                            print(type(conf))
                            info = {"class":c,"ID":float(id),"xCenter":xCenter, "yCenter":yCenter,"Radius":radius, "Area": float(width*height),"Confidence":float(conf.cpu().numpy())}
                            infoToSendfromYolo.append(info)

                          

                    LOGGER.info(f'{s}Done. YOLO:({t3 - t2:.3f}s), DeepSort:({t5 - t4:.3f}s)')

                else:
                    deepsort.increment_ages()
                    LOGGER.info('No detections')

        return infoToSendfromYolo

class track_ros(Node):
    def __init__(self):
        super().__init__('track_ros')

        self.pub_bboxes = self.create_publisher(BoundingBoxes, 'track/bounding_boxes',10)

        #Parameters
        FILE = Path(__file__).resolve()
        ROOT = FILE.parents[0]  # yolov5 deepsort root directory
        if str(ROOT) not in sys.path:
            sys.path.append(str(ROOT))  # add ROOT to PATH
        ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

        self.declare_parameter('yolo_model', str(ROOT) + '/yolo_ros/best_test_720p_4.pt')
        self.declare_parameter('deep_sort_model', str(ROOT) + '/config/osnet_ibn_x1_0_MSMT17')
        self.declare_parameter('imagesz', [640])
        self.declare_parameter('conf_thres', 0.5)
        self.declare_parameter('iou_thres', 0.5)
        self.declare_parameter('max_det', 1000)
        self.declare_parameter('device', 'cuda:0')
        self.declare_parameter('show_vid', False)
        self.declare_parameter('classes', None)
        self.declare_parameter('agnostic_nms', False)
        self.declare_parameter('half', True)
        self.declare_parameter('dnn', True)
        self.declare_parameter('augment', True)
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



        self.yolov5 = track(self.source,
                                self.yolo_model,
                                self.deep_sort_model,
                                self.config_deepsort,
                                self.visualize,
                                self.augment,
                                self.show_vid,
                                self.imgsz,
                                self.half,
                                self.dnn,
                                self.device,
                                self.max_det,
                                self.conf_thres,
                                self.iou_thres,
                                self.classes,
                                self.agnostic_nms,
                                )
    def tracker_callback(self):
        ###TODO: Read in the UDP Stream here as an image
        # image_raw = self.bridge.imgmsg_to_cv2(image, "bgr8")
        image =UDP
        # return (class_list, confidence_list, x_min_list, y_min_list, x_max_list, y_max_list)
        class_list, confidence_list, x_min_list, y_min_list, x_max_list, y_max_list = self.track.detect_callback(image_raw)

        ##TODO: Convert this data into the BoundingBoxes mesage
        msg =BoundingBoxes(class_list, confidence_list, x_min_list, y_min_list, x_max_list, y_max_list)
        self.pub_bbox.publish(msg)


        print("start ==================")
        print(class_list, confidence_list, x_min_list, y_min_list, x_max_list, y_max_list)
        print("end ====================")

def ros_main(args=None):
    rclpy.init(args=args)
    track_node = track_ros()
    rclpy.spin(track_node)
    track_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    ros_main()