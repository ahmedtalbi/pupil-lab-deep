from __future__ import print_function
import logging

import cv2
import numpy as np
from pyglui import ui
from pyglui.cygl.utils import draw_points, draw_polyline

from plugin import Plugin

logger = logging.getLogger(__name__)
from time import time, localtime, gmtime, strftime
from pyglui.cygl.utils import RGBA
import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from sensor_msgs.msg import Image
from std_msgs.msg import String, Time
from cv_bridge import CvBridge
import subprocess
import threading
from visualization_msgs.msg import Marker

import caffe
from caffe.model_libs import *
from google.protobuf import text_format

from PIL import Image
from PIL import ImageFilter
import math
import os
import shutil
import stat
import subprocess

import cv2
import numpy as np

#matplotlib inline

# Make sure that caffe is on the python path:
caffe_root = '../'  # this file is expected to be in {caffe_root}/examples
import os
os.chdir(caffe_root)
import sys
sys.path.insert(0, 'python')

import caffe
caffe.set_device(0)
caffe.set_mode_gpu()

from google.protobuf import text_format
from caffe.proto import caffe_pb2
class FRCNN_Detector:
    def __init__(self, labelmap_file= '/home/ahmed/development/SSD/caffe/data/coco/labelmap_coco.prototxt',
                 model_def = '/home/ahmed/development/FasterR-CNN/py-faster-rcnn/models/pascal_voc/COCO/faster_rcnn_alt_opt/coco_test.pt',
                 model_weights = '/home/ahmed/development/FasterR-CNN/py-faster-rcnn/data/faster_rcnn_models/coco_vgg16_faster_rcnn_final.caffemodel'):
        self.model_def = model_def
        self.model_weights = model_weights
        # load PASCAL VOC labels
        self.labelmap_file =labelmap_file
        self.transformer = None

        file = open(labelmap_file, 'r')
        self.labelmap = caffe_pb2.LabelMap()
        text_format.Merge(str(file.read()), self.labelmap)
        self.net = caffe.Net(self.model_def,      # defines the structure of the model
                self.model_weights,  # contains the trained weights
                caffe.TEST)     # use test mode (e.g., don't perform dropout)

        # input preprocessing: 'data' is the name of the input blob == net.inputs[0]
        self.transformer = caffe.io.Transformer({'data': self.net.blobs['data'].data.shape})
        self.transformer.set_transpose('data', (2, 0, 1))
        self.transformer.set_mean('data', np.array([104,117,123])) # mean pixel
        self.transformer.set_raw_scale('data', 255)  # the reference model operates on images in [0,255] range instead of [0,1]
        image_resize = 300
        self.net.blobs['data'].reshape(1,3,image_resize,image_resize)
        #transformer.set_channel_swap('data', (2,1,0))  # the reference model has channels in BGR order instead of RGB
    def get_labelname(self, labels):
        num_labels = len(self.labelmap.item)
        labelnames = []
        if type(labels) is not list:
            labels = [labels]
        for label in labels:
            found = False
            for i in xrange(0, num_labels):
                if label == self.labelmap.item[i].label:
                    found = True
                    labelnames.append(self.labelmap.item[i].display_name)
                    break
            assert found == True
        return labelnames


    def detect(self,frame, events):


        # set net to batch size of 1


        #image = caffe.io.load_image('/home/ahmed/development/SSD/caffe/examples/images/fish-bike.jpg')
        #image = caffe.io.load_image('/home/ahmed/Desktop/images2/Person.jpg')
        #image = caffe.io.load_image('/home/ahmed/Desktop/images2/Capture.jpg')
        #imCv=cv2.imread('/home/ahmed/Desktop/images2/cats.jpg')
        #cv2.namedWindow("preview")
        #imCv = cv2.VideoCapture(1)
        #image = caffe.io.load_image('/home/ahmed/Desktop/images2/cats.jpg')

        image = frame / 255.
        image = image[:,:,(2,1,0)]

        #image=caffe.io.array_to_datum(array)

        transformed_image = self.transformer.preprocess('data', image)
        self.net.blobs['data'].data[...] = transformed_image
        # Forward pass.
        detections = self.net.forward()['detection_out']

        # Parse the outputs.
        det_label = detections[0,0,:,1]
        det_conf = detections[0,0,:,2]
        det_xmin = detections[0,0,:,3]
        det_ymin = detections[0,0,:,4]
        det_xmax = detections[0,0,:,5]
        det_ymax = detections[0,0,:,6]

        # Get detections with confidence higher than 0.6.
        top_indices = [i for i, conf in enumerate(det_conf) if conf >= 0.6]

        top_conf = det_conf[top_indices]
        top_label_indices = det_label[top_indices].tolist()
        top_labels = self.get_labelname(top_label_indices)
        top_xmin = det_xmin[top_indices]
        top_ymin = det_ymin[top_indices]
        top_xmax = det_xmax[top_indices]
        top_ymax = det_ymax[top_indices]


        '''for i in xrange(top_conf.shape[0]):
            xmin = int(round(top_xmin[i] * frame.shape[1]))
            ymin = int(round(top_ymin[i] * frame.shape[0]))
            xmax = int(round(top_xmax[i] * frame.shape[1]))
            ymax = int(round(top_ymax[i] * frame.shape[0]))
            print(xmin, ymin, xmax, ymax)
            score = top_conf[i]
            label = int(top_label_indices[i])
            label_name = top_labels[i]
            display_txt[i] = '%s: %.2f'%(label_name, score)
            coords[i] = (xmin, ymin, xmax, ymax)'''
        return [top_xmin,top_ymin,top_xmax,top_ymax, top_labels, top_conf, top_label_indices]
