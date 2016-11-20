#!/usr/bin/env python

# --------------------------------------------------------

import sys

# add this path to make sure we find caffe and faster_rcnn
sys.path.append("/home/aicrobo/py-faster-rcnn/tools/")
import _init_paths
from fast_rcnn.config import cfg
from fast_rcnn.test import im_detect
from fast_rcnn.nms_wrapper import nms
import numpy as np
import caffe

CLASSES = ('__background__',
           'aeroplane', 'bicycle', 'bird', 'boat',
           'bottle', 'bus', 'car', 'cat', 'chair',
           'cow', 'diningtable', 'dog', 'horse',
           'motorbike', 'person', 'pottedplant',
           'sheep', 'sofa', 'train', 'tvmonitor')

NETS = {'vgg16': ('VGG16',
                  'VGG16_faster_rcnn_final.caffemodel'),
        'zf': ('ZF',
               'ZF_faster_rcnn_final.caffemodel')}


class Processor:
    def __init__(self):
        prototxt = '/home/aicrobo/py-faster-rcnn/models/pascal_voc/VGG16/faster_rcnn_alt_opt/faster_rcnn_test.pt'
        caffemodel = '/home/aicrobo/py-faster-rcnn/data/faster_rcnn_models/VGG16_faster_rcnn_final.caffemodel'

        caffe.set_mode_gpu()
        caffe.set_device(0)
        cfg.GPU_ID = 0
        cfg.TEST.HAS_RPN = True  # Use RPN for proposals

        self.net = caffe.Net(prototxt, caffemodel, caffe.TEST)

    def detect(self, im):
        scores, boxes = im_detect(self.net, im)

        conf_thresh = 0.8
        nms_thresh = 0.3
        result = []
        for cls_ind, cls in enumerate(CLASSES[1:]):
            cls_ind += 1  # because we skipped background
            cls_boxes = boxes[:, 4 * cls_ind:4 * (cls_ind + 1)]
            cls_scores = scores[:, cls_ind]
            dets = np.hstack((cls_boxes, cls_scores[:, np.newaxis])).astype(np.float32)
            keep = nms(dets, nms_thresh)
            dets = dets[keep, :]

            inds = np.where(dets[:, -1] >= conf_thresh)[0]

            for i in inds:
                # leftup corner x, y; bottomdown corner x, y
                bbox = dets[i, :4]
                score = dets[i, -1]

                instance = [bbox, score, cls]

                result.append(instance)

        return result


def process(im):
    cfg.TEST.HAS_RPN = True  # Use RPN for proposals

    prototxt = '/home/aicrobo/py-faster-rcnn/models/pascal_voc/VGG16/faster_rcnn_alt_opt/faster_rcnn_test.pt'
    caffemodel = '/home/aicrobo/py-faster-rcnn/data/faster_rcnn_models/VGG16_faster_rcnn_final.caffemodel'

    use_gpu = True
    if use_gpu:
        caffe.set_mode_gpu()
        caffe.set_device(0)
        cfg.GPU_ID = 0
    else:
        caffe.set_mode_cpu()
    net = caffe.Net(prototxt, caffemodel, caffe.TEST)

    scores, boxes = im_detect(net, im)

    conf_thresh = 0.8
    nms_thresh = 0.3
    result = []
    for cls_ind, cls in enumerate(CLASSES[1:]):
        cls_ind += 1  # because we skipped background
        cls_boxes = boxes[:, 4 * cls_ind:4 * (cls_ind + 1)]
        cls_scores = scores[:, cls_ind]
        dets = np.hstack((cls_boxes, cls_scores[:, np.newaxis])).astype(np.float32)
        keep = nms(dets, nms_thresh)
        dets = dets[keep, :]

        inds = np.where(dets[:, -1] >= conf_thresh)[0]

        for i in inds:
            # leftup corner x, y; bottomdown corner x, y
            bbox = dets[i, :4]
            score = dets[i, -1]

            instance = [bbox, score, cls]
            result.append(instance)
            # result += str(int(bbox[0])) + ',' + str(int(bbox[1])) + ',' + str(int(bbox[2])) + ',' + str(
            #     int(bbox[3])) + ',' + str(cls) + ','

    print result
    return result
