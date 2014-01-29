#!/usr/bin/env python
__author__ = 'Rolf Jagerman'

from PySide import QtGui, QtCore
import os
from loadui import loadUi
from config import UI_DIRECTORY
from ros_thread import invoke_in_ros_thread, get_ros_thread
from drawers import Drawers
from aidu_gui.msg import Solenoid


class OpenDrawer(QtGui.QFrame):
    """
    The add delivery form that enables a user to place a delivery in the robot
    """

    def __init__(self, content):
        super(OpenDrawer, self).__init__()
        Drawers.setup()
        self.content = content
        loadUi(os.path.join(UI_DIRECTORY, 'open_drawer.ui'), self)
        self.no_button.clicked.connect(self.back)
        self.yes_button.clicked.connect(self.save)
        self.count = 10
        self.open_drawer = False
        self.drawer_id = '1'
        self.recipient_id = ''
        self.location_id = ''
        self.sender_id = ''
        self.success_message = 'Have you successfully placed your delivery?'
        self.instruction_message = 'place'

    def show(self, *args, **kwargs):
        super(OpenDrawer, self).show()
        self.count = 3
        self.no_button.setVisible(False)
        self.yes_button.setVisible(False)
        self.label_title.setText('Drawer %s is open' % self.drawer_id)
        self.label_description.setText('Please open drawer %s and %s your delivery' % (self.drawer_id, self.instruction_message))
        self.label_countdown.setText('...')
        self.timer = QtCore.QTimer()
        self.connect(self.timer, QtCore.SIGNAL("timeout()"), self.count_decrement)
        self.timer.start(1000)
        self.open_drawer = True
        invoke_in_ros_thread(self.solenoid)

    def solenoid(self):
        solenoid = Solenoid()
        solenoid.solenoid_number = int(self.drawer_id)
        get_ros_thread().solenoid_publisher.publish(solenoid)

    def count_decrement(self):
        self.count -= 1
        self.label_countdown.setText('.' * (3 - self.count % 3))
        if self.count <= 0 and self.timer is not None:
            self.timer.stop()
            self.label_countdown.setText('')
            self.label_description.setText(self.success_message)
            self.no_button.setVisible(True)
            self.yes_button.setVisible(True)

    def save(self):
        pass # self.content.activate(self.content.components['welcome'])

    def back(self):
        pass # self.content.activate(self.content.components['add_delivery'])
