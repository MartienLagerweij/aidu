#!/usr/bin/env python
__author__ = 'Rolf Jagerman'

from PySide import QtGui
import os
from authentication import AuthenticationListener, AuthenticationClient
from loadui import loadUi
from config import UI_DIRECTORY
from drawers import Drawers


class RetrieveDelivery(QtGui.QFrame, AuthenticationListener):
    """
    The retrieve delivery form that enables a user to receive a delivery from the robot
    """

    def __init__(self, content):
        super(RetrieveDelivery, self).__init__()
        self.content = content
        loadUi(os.path.join(UI_DIRECTORY, 'receive_delivery.ui'), self)
        self.drawer1_button.clicked.connect(self.drawer1_open)
        self.drawer2_button.clicked.connect(self.drawer2_open)
        self.drawer3_button.clicked.connect(self.drawer3_open)
        self.cancel_button.clicked.connect(self.cancel)
        self.recipient_id = None
        self.drawer_id = '1'
        AuthenticationClient.add_listener(self)

    def show(self, *args, **kwargs):
        super(RetrieveDelivery, self).show()
        self.disable_drawers()
        self.prepare_open_drawer()
        for drawer in Drawers.receive_drawers(self.recipient_id):
            if drawer == '1':
                self.drawer1_button.setEnabled(True)
            elif drawer == '2':
                self.drawer2_button.setEnabled(True)
            elif drawer == '3':
                self.drawer3_button.setEnabled(True)

    def prepare_open_drawer(self):
        self.content.components['open_drawer'].save = lambda : self.save()
        self.content.components['open_drawer'].back = lambda : self.back()
        self.content.components['open_drawer'].success_message = 'Have you succesfully received your delivery?'
        self.content.components['open_drawer'].instruction_message = 'retrieve'

    def on_login(self, user):
        self.recipient_id = user.id

    def on_login_failure(self, user):
        self.recipient_id = None
        self.disable_drawers()

    def on_logout(self, user):
        self.disable_drawers()

    def back(self):
        self.content.activate(self.content.components['retrieve_delivery'])

    def save(self):
        Drawers.remove_delivery(self.drawer_id)
        self.content.activate(self.content.components['welcome'])

    def disable_drawers(self):
        self.drawer1_button.setEnabled(False)
        self.drawer2_button.setEnabled(False)
        self.drawer3_button.setEnabled(False)

    def cancel(self):
        self.content.activate(self.content.components['welcome'])

    def drawer1_open(self):
        self.drawer_id = '1'
        self.content.activate(self.content.components['open_drawer'])

    def drawer2_open(self):
        self.drawer_id = '2'
        self.content.activate(self.content.components['open_drawer'])

    def drawer3_open(self):
        self.drawer_id = '3'
        self.content.activate(self.content.components['open_drawer'])