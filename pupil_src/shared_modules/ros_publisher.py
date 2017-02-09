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



class ROS_Publisher(Plugin):
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

    def __init__(self, g_pool, publish=False, roscore=False):
        super(ROS_Publisher, self).__init__(g_pool)
        self._alive = True
        self.g_pool = g_pool


        self.running = True

        self.img_shape = None
        self._window = None
        self.start_time = time()
        self.m_to_screen = None
        self.menu = None
        self.button = None
        self.add_button = None

        self.publish = publish
        self.pupil_display_list = []

        self.template_image = None
        self.bottom_right = None
        self.top_left = None

        self.timestamps = []
        self.data = {'pupil_positions': [], 'gaze_positions': [], 'notifications': []}
        self.frame_count = 0
        self.roscore = roscore
        self.rosbag = None


        '''if self.bag_file:
            self.rosbag = subprocess.Popen('rosbag record -a', stdin=subprocess.PIPE, shell=True, cwd='/home/ahmed/Desktop/TestBag/')'''

        if self.publish:
            self.roscore = subprocess.Popen('roscore')


    def init_gui(self):
        '''
        needs to be implemented as optional
        if the app allows a gui, you may initalize your part of it here.
        '''
        self.menu = ui.Growing_Menu('Image Manipulation')
        self.g_pool.sidebar.append(self.menu)

        self.button = ui.Thumb('running', self, label='K', hotkey='k')
        #self.button.on_color[:] = (1., .2, .4, .8)
        #self.g_pool.quickbar.append(self.button)

        # Next line will use objects instead of surfaces je
        # self.add_button = ui.Thumb('add_surface',setter=self.add_surface,getter=lambda:False,label='A',hotkey='a')
        # self.g_pool.quickbar.append(self.add_button)
        self.update_gui_markers()

    def update_gui_markers(self):

        def close():
            self.alive = False
        self.menu.elements[:] = []
        self.menu.append(ui.Button('Close', close))
        self.menu.append(ui.Info_Text(
            'This plugin launches a roscore session and three rostopics: Frames, Gaze positions and confidences and a Timestamp'))
        # self.menu.append(ui.Switch('robust_detection',self,label='Robust detection'))
        # self.menu.append(ui.Switch('invert_image',self,label='Use inverted markers'))
        self.menu.append(ui.Switch('roscore', self, label='Launch ROS Core'))
        self.menu.append(ui.Switch('publish', self, label='ROS Publisher'))

        # self.menu.append(ui.Button("Add surface", lambda:self.add_surface('_'),))#to implement with multiple objects

    def gl_display(self):
        """
        gets called once every frame when its time to draw onto the gl canvas.
        """
        pass

    def get_init_dict(self):
        return {'roscore': self.roscore, 'publish': self.publish, 'bag_file':self.bag_file}

    def get_rec_time_str(self):
        rec_time = gmtime(time() - self.start_time)
        return strftime("%H:%M:%S", rec_time)

    def ros_talker(self,frame,events):
        self.button.status_text = self.get_rec_time_str()
        pub_gaze = rospy.Publisher('GazePos', numpy_msg(Floats), queue_size=10)
        pub_frame = rospy.Publisher('Frame', Image, queue_size=10)
        pub_time_frame = rospy.Publisher('Frame_timestamp', Time, queue_size=10)


        rate = rospy.Rate(100)  # 10hz
        if self.publish:
            for pt in events.get('gaze_positions', []):
                msg = np.asarray(memoryview(np.asarray(frame.jpeg_buffer)))
                cv2.CV_LOAD_IMAGE_COLOR = 1
                img2 = np.asarray(cv2.imdecode(msg, cv2.CV_LOAD_IMAGE_COLOR))
                rospy.loginfo('GazePos')
                pt2 = np.array([pt['norm_pos'][0],pt['norm_pos'][1], pt['confidence'], pt['timestamp']], dtype=np.float32)
                pub_gaze.publish(pt2)
                msg_frame = CvBridge().cv2_to_imgmsg(img2, "bgr8")
                pub_frame.publish(msg_frame)
                pub_time_frame.publish(frame.timestamp)
                rate.sleep()

    def update(self, frame, events, thr=0.5):
        self.img_shape = frame.height, frame.width, 3

        if self.running:

            if not self.roscore:
                try:
                    self.roscore.send_signal(subprocess.signal.SIGINT)
                except AttributeError:
                    pass

            if self.publish:
                try:
                    rospy.init_node('ros_talker', anonymous=True)
                    '''thread = threading.Thread(target=self.ros_talker, args = (frame,events))
                    thread.start()
                    thread.join()'''
                    self.ros_talker(frame,events)
                except rospy.ROSInterruptException:
                    print 'error ROS'
                    pass
                except threading.ThreadError:
                    print 'error Threading'
                    pass

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
        pass

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
