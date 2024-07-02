python train.py --img 640 --epochs 10 --data blimps/data.yaml --weights yolov5s.pt 
python export.py --weights best.pt --img 640 --batch 1 --include onnx --opset 12
python onnx2rknn.py bestblimps.onnx bestblimps.rknn rk3588
scp -r best.rknn opi@opi1:/home/opi/rknn-multi-threaded-3588/rknnModel
python detect.py --weights yolov5s.pt --source 0 