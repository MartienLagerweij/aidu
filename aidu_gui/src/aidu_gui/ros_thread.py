#!/usr/bin/env python
__author__ = 'Rolf Jagerman'

import sys

import rospy
from PySide.QtCore import QThread

from aidu_user_management.msg import Authentication
from aidu_gui.msg import Solenoid
from authentication import AuthenticationClient
import manager


class ROSThread(QThread):
    """
    This thread handles the ROS logic outside of the main GUI thread.
    This is necessary because we are running a GUI toolkit and therefore rospy cannot handle any signals from the main
    thread.
    """
    def __init__(self, *args, **kwargs):
        super(ROSThread, self).__init__(*args, **kwargs)
        self.done = False
        self.running = True
        self.user_management = None
        self.listeners = []
        self.functions = []

    def run(self, *args, **kwargs):
        """
        Correctly initialize ROS, add a callback handler for shutdown and keep running ROS until either the GUI shuts
        down or rospy shuts down.
        """

        rospy.init_node("main", disable_signals=True)
        rospy.myargv(argv=sys.argv)
        rospy.on_shutdown(self.stop)

        #rospy.wait_for_service('authenticate')
        #AuthenticationClient.login = rospy.ServiceProxy('authenticate', Authenticate)
        self.solenoid_publisher = rospy.Publisher("solenoid", Solenoid)
        login_subscriber = rospy.Subscriber('authentication', Authentication, AuthenticationClient.login_handler)

        while not rospy.is_shutdown() and self.running:
            try:
                if len(self.functions) > 0:
                    func, args, kwargs = self.functions.pop()
                    try:
                        func(*args, **kwargs)
                    except Exception as e:
                        rospy.logerr(e.message)
                rospy.sleep(0.1)
            except rospy.ROSInterruptException:
                break

        rospy.signal_shutdown("is_shutdown() evaluated to True" if self.running else "ROS thread manually stopped")
        self.done = True

    def stop(self):
        """
        Attempts to stop the ROS thread. This does not guarantee the thread will immediately stop. Check self.done
        to verify when the thread stops.
        """
        self.running = False


def invoke_in_ros_thread(fn, *args, **kwargs):
    manager.Manager.ros_thread.functions.append((fn, args, kwargs))


def get_ros_thread():
    return manager.Manager.ros_thread