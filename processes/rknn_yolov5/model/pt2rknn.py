import argparse
import sys
from rknn.api import RKNN

DATASET_PATH = None
QUANT = False

def parse_arg():

    if len(sys.argv) == 4:
        model_path = sys.argv[1]
        output_path = sys.argv[2]
        platform = sys.argv[3]
    else:
        print("Check your command, usage: python3 pt2rknn.py [model_path] [output_path] [platform]")
        exit(1)
    
    return model_path, output_path, platform

if __name__ == '__main__':
    model_path, output_path, platform = parse_arg()
    rknn = RKNN(verbose=False)

    print('--> Config model')
    rknn.config(mean_values=[[0, 0, 0]], std_values=[[255, 255, 255]], target_platform=platform)
    print('done')

    print('--> Loading model')
    ret = rknn.load_pytorch(model=model_path, input_size_list=[[1, 3, 640, 640]])
    if ret != 0:
        print('Load model failed!')
        exit(ret)
    print('done')

    print('--> Building model')
    if QUANT:
        ret = rknn.build(do_quantization=QUANT, dataset=DATASET_PATH)
    else:
        ret = rknn.build(do_quantization=QUANT)
    if ret != 0:
        print('Build model failed!')
        exit(ret)
    print('done')

    print('--> Export rknn model')
    ret = rknn.export_rknn(output_path)
    if ret != 0:
        print('Export rknn model failed!')
        exit(ret)
    print('done')

    # Release
    rknn.release()