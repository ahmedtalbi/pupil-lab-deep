'''
(*)~----------------------------------------------------------------------------------
 Pupil - eye tracking platform
 Copyright (C) 2012-2016  Pupil Labs

 Distributed under the terms of the GNU Lesser General Public License (LGPL v3.0).
 License details are in the file license.txt, distributed as part of this software.
----------------------------------------------------------------------------------~(*)
'''
import logging

import cv2
import numpy as np
from pyglui import ui, graph
from pyglui.cygl.utils import draw_points, draw_polyline, RGBA

from plugin import Plugin

logger = logging.getLogger(__name__)
from time import time, localtime, gmtime, strftime
import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from sensor_msgs.msg import Image
from std_msgs.msg import String, Time
from cv_bridge import CvBridge
import subprocess
import threading
from visualization_msgs.msg import Marker
from Detection_SSD import SSD_Detector
#from Detection_Faster_RCNN import FRCNN_Detector


'''
A simple example Plugin: 'display_recent_gaze.py'
It is a good starting point to build your own plugin.
'''


class Object_Detection_SSD(Plugin):
    """docstring for Plugin
    plugin is a base class
    it has all interfaces that will be called
    instances of this class usually get added to a plugins list
    this list will have its members called with all methods invoked.

    """
    # if you have a plugin that can exist multiple times make this false in your derived class
    uniqueness = 'by_class'
    # uniqueness = 'not_unique'
    # uniqueness = 'by_base_class'

    # between 0 and 1 this indicated where in the plugin excecution order you plugin lives:
    # <.5  are things that add/mofify information that will be used by other plugins and rely on untouched data.
    # You should not edit frame if you are here!
    # == .5 is the default.
    # >.5 are things that depend on other plugins work like display , saving and streaming
    # you can change this in __init__ for your instance or in the class definition
    order = .5

    def __init__(self, g_pool, crop_image=False, detect=False, template_path='Please enter a valid path', ratio=0.5,
                 template_method='cv2.TM_SQDIFF', mode='Static Template'):
        super(Object_Detection_SSD, self).__init__(g_pool)
        self._alive = True
        self.g_pool = g_pool
        self.coord = None
        self.mode = mode
        self.running = True
        self.detect = detect
        self.detector= SSD_Detector()
        #self.detector= FRCNN_Detector()
        self.img_shape = None
        self._window = None
        self.start_time = time()
        self.m_to_screen = None
        self.menu = None
        self.button = None
        self.add_button = None
        self.ratio = ratio
        self.pupil_display_list = []

        self.bottom_right = None
        self.top_left = None
        self.crop_image = crop_image
        self.timestamps = []
        self.data = {'pupil_positions': [], 'gaze_positions': [], 'notifications': []}
        self.frame_count = 0


        self.coord= None
        self.disp_text = None
        self.img= None
        self.Croppedimage = None
        self.norm_coord = None

        self.dimensions = None

    def init_gui(self):
        '''
        needs to be implemented as optional
        if the app allows a gui, you may initalize your part of it here.
        '''
        self.menu = ui.Growing_Menu('Image Manipulation')
        self.g_pool.sidebar.append(self.menu)

        self.button = ui.Thumb('running', self, label='OD', hotkey='o')
        self.button.on_color[:] = (1., .2, .4, .8)
        self.g_pool.quickbar.append(self.button)
        # Next line will use objects instead of surfaces je
        # self.add_button = ui.Thumb('add_surface',setter=self.add_surface,getter=lambda:False,label='A',hotkey='a')
        # self.g_pool.quickbar.append(self.add_button)
        self.update_gui_markers()

    def update_gui_markers(self):

        def close():
            self.alive = False

        def set_template(new_path):
            self.template_image = cv2.imread(new_path, 0)
            if self.template_image == None:
                self.template_path = 'Please enter a valid path'
            else:
                self.template_path = new_path

        self.menu.elements[:] = []
        self.menu.append(ui.Button('Close', close))
        self.menu.append(ui.Info_Text(
            'This plugin uses the SSD Algorithm implementation for object Detection'))
        # self.menu.append(ui.Switch('robust_detection',self,label='Robust detection'))
        # self.menu.append(ui.Switch('invert_image',self,label='Use inverted markers'))
        self.menu.append(ui.Slider('ratio', self, step=0.05, min=0.1, max=1))
        self.menu.append(ui.Switch('crop_image', self, label='Crop Image'))
        self.menu.append(ui.Switch('detect', self, label='detector'))

    def gl_display(self):
        """
        gets called once every frame when its time to draw onto the gl canvas.
        """
        if self.coord != None:
            [top_xmin,top_ymin,top_xmax,top_ymax, top_labels, top_conf, top_label_indices] = self.coord
            for i in xrange(top_conf.shape[0]):
                xmin = int(round(top_xmin[i] * self.img.shape[1]))
                ymin = int(round(top_ymin[i] * self.img.shape[0]))
                xmax = int(round(top_xmax[i] * self.img.shape[1]))
                ymax = int(round(top_ymax[i] * self.img.shape[0]))

                score = top_conf[i]
                label = int(top_label_indices[i])
                label_name = top_labels[i]
                display_txt = 'detected a %s: %.2f'%(label_name, score)
                coords= (xmin, ymin, xmax, ymax)
                '''if self.Croppedimage != None:
                    top_left = (coords[0]+self.dimensions[0],coords[1]+self.dimensions[1])
                    bottom_left = (coords[0]+self.dimensions[0],coords[3]+self.dimensions[3])
                    bottom_right = (coords[2]+self.dimensions[2],coords[3]+self.dimensions[3])
                    top_right = (coords[2]+self.dimensions[2],coords[1]+self.dimensions[1])
                    draw_polyline([top_left, top_right, bottom_right, bottom_left, top_left], color=RGBA(1.,.2,.4,.9), thickness=3)
                    logger.info(display_txt)'''
                coord_center = [0.5*(coords[0]+coords[2]),0.5*(coords[1]+coords[3])]
                coord_center = (int(coord_center[0]),int(coord_center[1]))
                print 'coords_center'
                print coord_center
                if self.dimensions == None:
                    break
                elif self.dimensions[0]<coord_center[0]<self.dimensions[1] and self.dimensions[2]<coord_center[1] and coord_center[1]<self.dimensions[3]:
                    print 'hello!'
                    top_left = (coords[0],coords[1])
                    bottom_left = (coords[0],coords[3])
                    bottom_right = (coords[2],coords[3])
                    top_right = (coords[2],coords[1])
                    draw_polyline([top_left, top_right, bottom_right, bottom_left, top_left], color=RGBA(1.,.2,.4,.9), thickness=3)
                    logger.info(display_txt)

            '''draw_points([self.pupil_display_list], size=35, color=RGBA(.1, .2, 1., .8))
            w, h = self.template_image.shape[::-1]
            bottom_left = (self.top_left[0], self.top_left[1] + h)
            top_right = (self.top_left[0] + w, self.top_left[1])
            draw_polyline([self.top_left, top_right, self.bottom_right, bottom_left, self.top_left],
                          color=RGBA(.5, .3, .6, .5), thickness=3)'''

    def get_init_dict(self):
        return {'mode': self.mode, 'detect': self.detect,  'ratio': self.ratio, 'crop_image': self.crop_image}

    def get_rec_time_str(self):
        rec_time = gmtime(time() - self.start_time)
        return strftime("%H:%M:%S", rec_time)

    def update(self, frame, events, thr=0.5):
        self.img_shape = frame.height, frame.width, 3
        #print('image shape: '+ str(self.img_shape))

        # implement function to crop the image centred on the pupil positions
        # move to SearchedObject class
        def _remove_borders(src, top, bottom, left, right):
            if top < 0 or left < 0 or right < 0 or bottom < 0:
                self.crop_image = False
            else:
                size = bottom - top, right - left
                out = np.zeros(size, dtype=np.uint8)
                out = src[top:bottom, left:right]
                return out

        def _dim(ratio, pt, height, width):
            p = max(min(pt[0],1),0),max(min(pt[1],1),0)
            y_max = int(min(p[1]*height + int(height * ratio/2), height))
            y_min = int(max(p[1]*height-int(height * ratio/2), 0))
            x_min = int(max(p[0]*width -int(width * ratio/2),0))
            x_max = int(min(p[0]*width + int(width * ratio/2), width))
            return x_min,x_max,y_min,y_max

        if self.running:
            msg = np.asarray(memoryview(np.asarray(frame.jpeg_buffer)))
            cv2.CV_LOAD_IMAGE_COLOR = 1;
            self.img = cv2.imdecode(msg, cv2.CV_LOAD_IMAGE_COLOR)

            for pt in events.get('gaze_positions', []):
                self.norm_coord=pt['norm_pos']
                self.dimensions=_dim(self.ratio,self.norm_coord,frame.height,frame.width)
                self.coord = self.detector.detect(self.img,events)

    def on_click(self, pos, button, action):
        """
        gets called when the user clicks in the window screen
        """
        pass

    def on_window_resize(self, window, w, h):
        '''
        gets called when user resizes window.
        window is the glfw window handle of the resized window.
        '''
        pass

    def on_notify(self, notification):
        """
        this gets called when a plugin wants to notify all others.
        notification is a dict in the format {'subject':'notification_category.notification_name',['addional_field':'blah']}
        implement this fn if you want to deal with notifications
        note that notifications are collected from all threads and processes and dispatched in the update loop.
        this callback happens in the main thread.
        """
        pass

    ## if you want a session persistent plugin implement this function:
    def get_init_dict(self):
        raise NotImplementedError()
        # d = {}
        # # add all aguments of your plugin init fn with paramter names as name field
        # # do not include g_pool here.
        # return d

    def cleanup(self):
        """
        gets called when the plugin get terminated.
        This happens either voluntarily or forced.
        if you have an gui or glfw window destroy it here.
        """
        self.notify_all({'subject':'template_matching.stopped'})
        if self.menu:
            self.g_pool.sidebar.remove(self.menu)
            self.menu = None


    ###do not change methods,properties below this line in your derived class

    def notify_all(self, notification):
        """

        Do not overwrite this method.

        Call `notify_all` to notify all other plugins and processes with a notification:

        notification is a dict in the format {'subject':'notification_category.[subcategory].action_name',['addional_field':'foo']}

            adding 'timestamp':self.g_pool.get_timestamp() will allow other plugins to know when you created this notification.

            adding 'record':True will make recorder save the notification during recording.

            adding 'remote_notify':'all' will send the event all other pupil sync nodes in the same group.
            (Remote notifyifactions are not be recevied by any local actor.)

            adding 'remote_notify':node_UUID will send the event the pupil sync nodes with node_UUID.
            (Remote notifyifactions are not be recevied by any local actor.)

            adding 'delay':3.2 will delay the notification for 3.2s.
            If a new delayed notification of same subject is sent before 3.2s have passed we will discard the former notification.

        You may add more fields as you like.

        All notifications must be serializable

        """
        if self.g_pool.app in ('player', 'exporter'):
            if notification.get('delay', 0):
                notification['_notify_time_'] = time() + notification['delay']
                self.g_pool.delayed_notifications[notification['subject']] = notification
            else:
                self.g_pool.notifications.append(notification)
        else:
            self.g_pool.ipc_pub.notify(notification)
