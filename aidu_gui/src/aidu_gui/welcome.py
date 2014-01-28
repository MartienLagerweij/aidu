__author__ = 'Rolf Jagerman'

import os
from PySide import QtGui
from loadui import loadUi
from config import UI_DIRECTORY
from authentication import AuthenticationClient, AuthenticationListener


class Welcome(QtGui.QFrame, AuthenticationListener):
    """
    The add delivery form that enables a user to place a delivery in the robot
    """

    def __init__(self, content):
        super(Welcome, self).__init__()
        self.content = content
        loadUi(os.path.join(UI_DIRECTORY, 'welcome.ui'), self)
        AuthenticationClient.add_listener(self)

    def on_login(self, user):
        self.welcome_label.setText('Welcome %s' % user.first_name)
        self.add_delivery_button.clicked.connect(self.add_delivery)

    def on_logout(self, user):
        self.welcome_label.setText('Welcome')

    def add_delivery(self):
        self.content.activate(self.content.components['add_delivery'])
