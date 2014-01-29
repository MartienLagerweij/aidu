__author__ = 'Rolf Jagerman'

import os
from PySide import QtGui
from loadui import loadUi
from config import UI_DIRECTORY
from authentication import AuthenticationClient, AuthenticationListener
from drawers import Drawers


class Welcome(QtGui.QFrame, AuthenticationListener):
    """
    The add delivery form that enables a user to place a delivery in the robot
    """

    def __init__(self, content):
        super(Welcome, self).__init__()
        self.content = content
        loadUi(os.path.join(UI_DIRECTORY, 'welcome.ui'), self)
        AuthenticationClient.add_listener(self)
        self.user_id = ''
        self.add_delivery_button.clicked.connect(self.add_delivery)
        self.receive_delivery_button.clicked.connect(self.retrieve_delivery)

    def on_login(self, user):
        self.user_id = user.id
        self.welcome_label.setText('Welcome %s' % user.first_name)

    def on_logout(self, user):
        self.welcome_label.setText('Welcome')

    def add_delivery(self):
        if len(Drawers.available_drawers()) == 0:
            self.content.activate(self.content.components['error'])
            self.content.components['error'].message_label.setText('No more empty drawers available')
            self.content.activate_after(self.content.components['welcome'], 3500)
        else:
            self.content.activate(self.content.components['add_delivery'])

    def retrieve_delivery(self):
        if len(Drawers.receive_drawers(self.user_id)) == 0:
            self.content.activate(self.content.components['error'])
            self.content.components['error'].message_label.setText('You have no pending deliveries')
            self.content.activate_after(self.content.components['welcome'], 3500)
        else:
            self.content.activate(self.content.components['retrieve_delivery'])
