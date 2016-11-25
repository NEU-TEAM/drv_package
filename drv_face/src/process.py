#!/usr/bin/env python

# --------------------------------------------------------

import sys
import os
import numpy as np
import caffe

# Load caffe
caffe_root = '/home/aicrobo/caffe/'
sys.path.insert(0, caffe_root + 'python')


def process(im):
    result = [0, 0]
    prototxt = '/home/aicrobo/caffe/models/finetune_neu_face/neu_face_deploy.prototxt'
    caffemodel = '/home/aicrobo/caffe/models/finetune_neu_face/finetune_neu_face.caffemodel'

    if os.path.isfile(prototxt) and os.path.isfile(caffemodel):
        print 'caffe prototxt and model found.'
    else:
        print 'caffe prototxt and model not found!'
        return result

    use_gpu = True
    if use_gpu:
        caffe.set_mode_gpu()
        caffe.set_device(0)
    else:
        caffe.set_mode_cpu()
    net = caffe.Net(prototxt, caffemodel, caffe.TEST)

    # create transformer for the input called 'data'
    transformer = caffe.io.Transformer({'data': net.blobs['data'].data.shape})
    transformer.set_transpose('data', (2, 0, 1))  # move image channels to outermost dimension
    transformer.set_raw_scale('data', 255)  # rescale from [0, 1] to [0, 255]
    transformer.set_channel_swap('data', (2, 1, 0))  # swap channels from RGB to BGR

    net.blobs['data'].reshape(1,  # batch size
                              3,  # 3-channel (BGR) images
                              224, 224)  # image size is 224x224 for vgg face

    transformed_image = transformer.preprocess('data', im)
    # copy the image data into the memory allocated for the net
    net.blobs['data'].data[...] = transformed_image

    # perform classification
    output = net.forward()
    output_prob = output['prob'][0]  # the output probability vector for the first image in the batch

    # id and probability
    result = [output_prob.argmax(), output_prob[output_prob.argmax()]]

    print result
    return result
