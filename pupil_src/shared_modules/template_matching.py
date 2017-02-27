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
from pyglui import ui
from pyglui.cygl.utils import draw_points, draw_polyline

from plugin import Plugin

logger = logging.getLogger(__name__)
from time import time, localtime, gmtime, strftime
from pyglui.cygl.utils import RGBA, draw_points_norm
import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from sensor_msgs.msg import Image
from std_msgs.msg import String, Time
from cv_bridge import CvBridge
import subprocess
import threading
from visualization_msgs.msg import Marker
from matplotlib import pyplot as plt
'''
A simple example Plugin: 'display_recent_gaze.py'
It is a good starting point to build your own plugin.
'''


class Template_Matching(Plugin):
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

    def __init__(self, g_pool, save_image=False, template_path='Image name',
                 template_method='cv2.TM_SQDIFF', mode='Static Template'):
        super(Template_Matching, self).__init__(g_pool)
        self._alive = True
        self.g_pool = g_pool

        self.mode = mode
        self.running = True

        self.img_shape = None
        self._window = None
        self.start_time = time()
        self.m_to_screen = None
        self.menu = None
        self.button = None
        self.add_button = None
        self.template_method = template_method
        self.pupil_display_list = []
        self.template_path = template_path
        self.template_image = None
        self.bottom_right = None
        self.top_left = None
        self.timestamps = []
        self.data = {'pupil_positions': [], 'gaze_positions': [], 'notifications': []}
        self.frame_count = 0
        self.save_image = save_image
        self.Pupil_Rfolder = '/home/ahmed/eye-tracker/'

    def init_gui(self):
        '''
        needs to be implemented as optional
        if the app allows a gui, you may initalize your part of it here.
        '''
        self.menu = ui.Growing_Menu('Template Matching')
        self.g_pool.sidebar.append(self.menu)
        self.update_gui_markers()

    def update_gui_markers(self):

        def close():
            self.alive = False

        def set_template(new_path='tasse'):
            path=self.Pupil_Rfolder + 'templates/' + new_path + '.jpg'
            self.template_image = cv2.imread(path, 0)
            if self.template_image == None:
                self.template_path = 'Image Name'
            else:
                self.template_path = new_path

        self.menu.elements[:] = []
        self.menu.append(ui.Button('Close', close))
        self.menu.append(ui.Info_Text(
            'This plugin detects objects using a template matching method.'
            'You can also change the detection method by clicking the *template Method*'
            'Please adjust your Pupil Folder path and put the templates you want to match to in $Root_Folder/templates/'))
        # self.menu.append(ui.Switch('robust_detection',self,label='Robust detection'))
        # self.menu.append(ui.Switch('invert_image',self,label='Use inverted markers'))
        self.menu.append(ui.Switch('save_image', self, label='Save Image'))
        self.menu.append(ui.Selector('template_method', self, label='Template Methode',
                                     selection=['cv2.TM_CCOEFF', 'cv2.TM_CCOEFF_NORMED', 'cv2.TM_CCORR',
                                                'cv2.TM_CCORR_NORMED', 'cv2.TM_SQDIFF', 'cv2.TM_SQDIFF_NORMED']))
        self.menu.append(ui.Selector('mode', self, label='Mode', selection=['Static Template']))
        self.menu.append(ui.Text_Input('template_path', self, setter=set_template, label='Template Image Name'))

    def gl_display(self):
        """
        gets called once every frame when its time to draw onto the gl canvas.
        """
        if self.template_image != None:
            draw_points([self.pupil_display_list], size=35, color=RGBA(.1, .2, 1., .8))
            w, h = self.template_image.shape[::-1]
            bottom_left = (self.top_left[0], self.top_left[1] + h)
            top_right = (self.top_left[0] + w, self.top_left[1])
            draw_polyline([self.top_left, top_right, self.bottom_right, bottom_left, self.top_left],
                          color=RGBA(.5, .3, .6, .5), thickness=3)

    def get_init_dict(self):
        return {'mode': self.mode, 'template_method': self.template_method,
                'template_image': self.template, 'save_image': self.save_image}

    def get_rec_time_str(self):
        rec_time = gmtime(time() - self.start_time)
        return strftime("%H:%M:%S", rec_time)

    def update(self, frame, events, thr=0.5):
        self.img_shape = frame.height, frame.width, 3
        if self.running:

            msg = np.asarray(memoryview(np.asarray(frame.jpeg_buffer)))
            cv2.CV_LOAD_IMAGE_COLOR = 1;
            img = cv2.imdecode(msg, 0)

            if self.save_image:
                imTosave = cv2.imdecode(msg, cv2.CV_LOAD_IMAGE_COLOR)
                cv2.imwrite(self.Pupil_Rfolder+"SavedImages/frameNewCV"+str(self.frame_count)+".jpg", imTosave)
                self.save_image = False
                self.frame_count+=1
            img2 = img.copy()

            # change template source to make it possible directly from image add an extra mode for this
            if self.template_image!=None:
                w, h = self.template_image.shape[::-1]
                # All the 6 methods for comparison in a list
                if self.mode == 'Static Template':
                    img = img2.copy()
                    method = eval(self.template_method)
                    res = cv2.matchTemplate(img, self.template_image, method)
                    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
                    # If the method is TM_SQDIFF or TM_SQDIFF_NORMED, take minimum
                    if method in [cv2.TM_SQDIFF, cv2.TM_SQDIFF_NORMED]:
                        self.top_left = min_loc
                    else:
                        self.top_left = max_loc
                    self.bottom_right = (self.top_left[0] + w, self.top_left[1] + h)
                    self.pupil_display_list = (self.top_left[0] + w / 2, self.top_left[1] + h / 2)

                bottom_right = (self.top_left[0] + w, self.top_left[1] + h)
                cv2.rectangle(img,self.top_left, bottom_right, 255, 2)


                draw_points_norm([(self.top_left[0] + w/2, self.top_left[1] + h/2)],
                        size=35,
                        color=RGBA(1.,.2,.4,.9))


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

    def close(self):
        self.alive = False

    def cleanup(self):
        self.notify_all({'subject':'template_matching.stopped'})
        if self.menu:
            self.g_pool.sidebar.remove(self.menu)
            self.menu = None

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
