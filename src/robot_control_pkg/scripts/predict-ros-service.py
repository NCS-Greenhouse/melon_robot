#/usr/bin/python3
# YOLOv5 🚀 by Ultralytics, AGPL-3.0 license
"""
Run YOLOv5 segmentation inference on images, videos, directories, streams, etc.

Usage - sources:
    $ python segment/predict.py --weights yolov5s-seg.pt --source 0                               # webcam
                                                                  img.jpg                         # image
                                                                  vid.mp4                         # video
                                                                  screen                          # screenshot
                                                                  path/                           # directory
                                                                  list.txt                        # list of images
                                                                  list.streams                    # list of streams
                                                                  'path/*.jpg'                    # glob
                                                                  'https://youtu.be/LNwODJXcvt4'  # YouTube
                                                                  'rtsp://example.com/media.mp4'  # RTSP, RTMP, HTTP stream

Usage - formats:
    $ python segment/predict.py --weights yolov5s-seg.pt                 # PyTorch
                                          yolov5s-seg.torchscript        # TorchScript
                                          yolov5s-seg.onnx               # ONNX Runtime or OpenCV DNN with --dnn
                                          yolov5s-seg_openvino_model     # OpenVINO
                                          yolov5s-seg.engine             # TensorRT
                                          yolov5s-seg.mlmodel            # CoreML (macOS-only)
                                          yolov5s-seg_saved_model        # TensorFlow SavedModel
                                          yolov5s-seg.pb                 # TensorFlow GraphDef
                                          yolov5s-seg.tflite             # TensorFlow Lite
                                          yolov5s-seg_edgetpu.tflite     # TensorFlow Edge TPU
                                          yolov5s-seg_paddle_model       # PaddlePaddle
"""

import argparse
import os
import platform
import sys
from pathlib import Path
import numpy 
import torch
from matplotlib import pyplot as plt

import rospkg, rospy
rospy.init_node('yolo_predictor',anonymous=False) #run only one script

ROBOT_CONTROL_PKG_PATH = rospkg.RosPack().get_path('robot_control_pkg')
YOLO_PATH = str(Path(ROBOT_CONTROL_PKG_PATH).resolve().parents[0]) + '/yolov5/'
ROOT = YOLO_PATH
if ROOT not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

if(not os.path.isfile(YOLO_PATH + "/yolov5x-seg-best.pt")):
    from subprocess import call
    call(["curl", "-L", "-o", 
        YOLO_PATH + "/yolov5x-seg-best.pt", 
        "https://drive.google.com/uc?id=1OPq60h2DIgtQYjETnGc8DUUqT5OZeLih&export=download&confirm=t"])
    rospy.loginfo("Model Downloaded.")

import hashlib
# https://stackoverflow.com/questions/16874598/how-do-i-calculate-the-md5-checksum-of-a-file-in-python
md5 = hashlib.md5(open(YOLO_PATH + "/yolov5x-seg-best.pt",'rb').read()).hexdigest()
if(str(md5) != 'c279305b0e88809b47f196d2e8f8ab3d'):
    quit()
rospy.loginfo("Model MD5:%s.", str(md5))


from ultralytics.utils.plotting import Annotator, colors, save_one_box

from models.common import DetectMultiBackend
from utils.dataloaders import IMG_FORMATS, VID_FORMATS, LoadImages, LoadScreenshots, LoadStreams
from utils.general import (LOGGER, Profile, check_file, check_img_size, check_imshow, check_requirements, colorstr, cv2,
                           increment_path, non_max_suppression, print_args, scale_boxes, scale_segments,
                           strip_optimizer)
from utils.segment.general import masks2segments, process_mask, process_mask_native
from utils.torch_utils import select_device, smart_inference_mode

import rospy, rospkg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import PointCloud2 as pc2
# OpenCV2 for saving an image
import cv2
from greenhouse_arm_pkg.srv import melon_det, melon_detRequest, melon_detResponse
from std_msgs.msg import Float64MultiArray, Int16, Float64
bridge = CvBridge()
sys.path.insert(0,rospkg.RosPack().get_path('robot_control_pkg') + "/src/hrl_geom/")


weights= YOLO_PATH + "/yolov5x-seg-best.pt"  # model.pt path(s)
source=ROOT / 'data/images'  # file/dir/URL/glob/screen/0(webcam)
topic='/camera/color/image_raw'
data=ROOT / 'data/coco128.yaml'  # dataset.yaml path
imgsz=(640, 640)  # inference size (height, width)
conf_thres=0.25  # confidence threshold
iou_thres=0.45  # NMS IOU threshold
max_det=1000  # maximum detections per image
device=''  # cuda device, i.e. 0 or 0,1,2,3 or cpu
view_img=False  # show results
save_txt=False  # save results to *.txt
save_conf=False  # save confidences in --save-txt labels
save_crop=False  # save cropped prediction boxes
nosave=False  # do not save images/videos
classes=None  # filter by class: --class 0, or --class 0 2 3
agnostic_nms=False  # class-agnostic NMS
augment=False  # augmented inference
visualize=False  # visualize features
update=False  # update all models
project=ROOT / 'runs/predict-seg'  # save results to project/name
name='exp'  # save results to project/name
exist_ok=False  # existing project/name ok, do not increment
line_thickness=3  # bounding box thickness (pixels)
hide_labels=False  # hide labels
hide_conf=False  # hide confidences
half=False  # use FP16 half-precision inference
dnn=False  # use OpenCV DNN for ONNX inference
vid_stride=1  # video frame-rate stride
retina_masks=False

img_pub = rospy.Publisher("melon_detection_result_img", Image, queue_size=10)

def detect(req:melon_detRequest):
    global weights, source, topic, data, imgsz, conf_thres, max_det, device, view_img, save_txt, save_conf
    global save_crop, nosave, classes, agnostic_nms, augment, visualize, update, project, name
    global exist_ok, line_thickness, hide_labels, hide_conf, half, dnn, vid_stride, retina_masks
    global bs, seen, windows, dt, stride

    response = melon_detResponse()
    current_image_msg:Image =  req.Image
    #convert ros image type to CV2 image type
    cv2_img = bridge.imgmsg_to_cv2(current_image_msg, "bgr8")
    im = cv2_img.transpose((2, 0, 1))[::-1]
    #get the timestamp as the name of this pic
    # filename = '/mnt/Data/greenhouse_cases/image-stream-temp/'+  str(current_image_msg.header.stamp) +'.jpg'
    #save image to a folder
    # cv2.imwrite(filename, cv2_img)

    #Load this image
    # dataset = LoadImages(filename, img_size=imgsz, stride=stride, auto=pt, vid_stride=vid_stride)
    
    # vid_path, vid_writer = [None] * bs, [None] * bs

    # for path, im, im0s, vid_cap, s in dataset:
        # print(len(dataset))
    
    with dt[0]:
        im = torch.from_numpy(im.copy()).to(model.device)
        im = im.half() if model.fp16 else im.float()  # uint8 to fp16/32
        im /= 255  # 0 - 255 to 0.0 - 1.0
        if len(im.shape) == 3:
            im = im[None]  # expand for batch dim

    # Inference
    with dt[1]:
        # visualize = increment_path(save_dir / Path(path).stem, mkdir=True) if visualize else False
        pred, proto = model(im, augment=augment, visualize=visualize)[:2]

    # NMS
    with dt[2]:
        pred = non_max_suppression(pred, conf_thres, iou_thres, classes, agnostic_nms, max_det=max_det, nm=32)

    # Second-stage classifier (optional)
    # pred = utils.general.apply_classifier(pred, classifier_model, im, im0s)

    # Process predictions
    for i, det in enumerate(pred):  # per image
        # seen += 1
        # if webcam:  # batch_size >= 1
        #     p, im0, frame = path[i], im0s[i].copy(), dataset.count
        #     s += f'{i}: '
        # else:
        #     p, im0, frame = path, im0s.copy(), getattr(dataset, 'frame', 0)
        # p = Path(p)  # to Path
        # save_path = str(save_dir / p.name)  # im.jpg
        # txt_path = str(save_dir / 'labels' / p.stem) + ('' if dataset.mode == 'image' else f'_{frame}')  # im.txt
        s = ''
        s += '%gx%g ' % im.shape[2:]  # print string
        

        # im0 = im.copy()
        # imc = im0.copy() if save_crop else im0  # for save_crop
        im0 = cv2_img.copy()
        annotator = Annotator(im0, line_width=line_thickness, example=str(names))
        if len(det):
            # print(det)
            if retina_masks:
                # scale bbox first the crop masks
                det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im0.shape).round()  # rescale boxes to im0 size
                masks = process_mask_native(proto[i], det[:, 6:], det[:, :4], im0.shape[:2])  # HWC
            else:
                masks = process_mask(proto[i], det[:, 6:], det[:, :4], im.shape[2:], upsample=True)  # HWC
                det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im0.shape).round()  # rescale boxes to im0 size

            # Segments
            # if save_txt:
            #     segments = [
            #         scale_segments(im0.shape if retina_masks else im.shape[2:], x, im0.shape, normalize=True)
            #         for x in reversed(masks2segments(masks))]

            # Print results
            for c in det[:, 5].unique():
                n = (det[:, 5] == c).sum()  # detections per class
                s += f"{n} {names[int(c)]}{'s' * (n > 1)}, "  # add to string

            # Mask plotting
            annotator.masks(
                masks,
                colors=[colors(x, True) for x in det[:, 5]],
                im_gpu=torch.as_tensor(im0, dtype=torch.float16).to(device).permute(2, 0, 1).flip(0).contiguous() /
                255 if retina_masks else im[i])

            # Write results
            # for j, (*xyxy, conf, cls) in enumerate(reversed(det[:, :6])):
            #     # print(xyxy,conf,cls)
            #     if save_txt:  # Write to file
            #         seg = segments[j].reshape(-1)  # (n,2) to (n*2)
            #         line = (cls, *seg, conf) if save_conf else (cls, *seg)  # label format
            #         with open(f'{txt_path}.txt', 'a') as f:
            #             f.write(('%g ' * len(line)).rstrip() % line + '\n')
            #     if save_img or save_crop or view_img:  # Add bbox to image
            #         c = int(cls)  # integer class
            #         label = None if hide_labels else (names[c] if hide_conf else f'{names[c]} {conf:.2f}')
            #         annotator.box_label(xyxy, label, color=colors(c, True))
            #         # annotator.draw.polygon(segments[j], outline=colors(c, True), width=3)
            #     if save_crop:
                    # save_one_box(xyxy, imc, file=save_dir / 'crops' / names[c] / f'{p.stem}.jpg', BGR=True)
            

            #Generate ROS result here
            # print("mask length=",len(masks))
            # print("masks=",masks) #type is tensor, and HOW to convert this to numpy?
            # response.Image_Masks
            for j, (*xyxy, conf, cls) in enumerate(reversed(det[:, :6])):
                my_msg = Float64MultiArray()  
                my_msg.data = masks[j].cpu().numpy().astype(float).ravel()
                response.Image_Masks.append(my_msg)
                response.Classes.append(Int16(int(cls.cpu().numpy())))
                response.Confidences.append(Float64(int(conf.cpu().numpy())))
            print(len(response.Image_Masks))


            # for mask in masks:
            #     print(mask.cpu().numpy()) #works!!


        # Stream results
        im0 = annotator.result()
        #publish result

        img_msg:Image = bridge.cv2_to_imgmsg(im0)
        img_pub.publish(img_msg)
        
        # if view_img:
        #     if platform.system() == 'Linux' and p not in windows:
        #         windows.append(p)
        #         cv2.namedWindow(str(p), cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)  # allow window resize (Linux)
        #         cv2.resizeWindow(str(p), im0.shape[1], im0.shape[0])
        #     cv2.imshow(str(p), im0)
        #     if cv2.waitKey(1) == ord('q'):  # 1 millisecond
                # exit()

        # Save results (image with detections)
        # if save_img:
        #     if dataset.mode == 'image':
        #         cv2.imwrite(save_path, im0)
        #     else:  # 'video' or 'stream'
        #         if vid_path[i] != save_path:  # new video
        #             vid_path[i] = save_path
        #             if isinstance(vid_writer[i], cv2.VideoWriter):
        #                 vid_writer[i].release()  # release previous video writer
        #             if vid_cap:  # video
        #                 fps = vid_cap.get(cv2.CAP_PROP_FPS)
        #                 w = int(vid_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        #                 h = int(vid_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        #             else:  # stream
        #                 fps, w, h = 30, im0.shape[1], im0.shape[0]
        #             save_path = str(Path(save_path).with_suffix('.mp4'))  # force *.mp4 suffix on results videos
        #             vid_writer[i] = cv2.VideoWriter(save_path, cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h))
        #         vid_writer[i].write(im0)

    # Print time (inference-only)
    LOGGER.info(f"{s}{'' if len(det) else '(no detections), '}{dt[1].dt * 1E3:.1f}ms")

    return response

if __name__ == '__main__':
    # global weights, source, topic, data, imgsz, conf_thres, max_det, device, view_img, save_txt, save_conf
    # global save_crop, nosave, classes, agnostic_nms, augment, visualize, update, project, name
    # global exist_ok, line_thickness, hide_labels, hide_conf, half, dnn, vid_stride, retina_masks

    #init ros node here
    # opt = parse_opt()
    
    check_requirements(ROOT / 'requirements.txt', exclude=('tensorboard', 'thop'))
    source = str(source)
    save_img = not nosave and not source.endswith('.txt')  # save inference images
    webcam = False

    
    # Directories
    save_dir = increment_path(Path(project) / name, exist_ok=exist_ok)  # increment run
    (save_dir / 'labels' if save_txt else save_dir).mkdir(parents=True, exist_ok=True)  # make dir

    # Load model
    device = select_device(device)
    model = DetectMultiBackend(weights, device=device, dnn=dnn, data=data, fp16=half)
    stride, names, pt = model.stride, model.names, model.pt
    imgsz = check_img_size(imgsz, s=stride)  # check image size

    # Dataloader
    bs = 1  # batch_size

    # Run inference
    model.warmup(imgsz=(1 if pt else bs, 3, *imgsz))  # warmup
    seen, windows, dt = 0, [], (Profile(), Profile(), Profile())

    rospy.Service('melon_detector_yolov5', melon_det, detect)

    
    while(not rospy.is_shutdown()):
        # current_image_msg:Image =  rospy.wait_for_message("/camera/color/image_raw",Image,timeout=600)
        rospy.Rate(0.5).sleep()
        




