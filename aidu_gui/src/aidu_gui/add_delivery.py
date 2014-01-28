#!/usr/bin/env python
__author__ = 'Rolf Jagerman'

from PySide import QtGui
import os
from authentication import AuthenticationListener, AuthenticationClient
from loadui import loadUi
from config import UI_DIRECTORY


class AddDelivery(QtGui.QFrame, AuthenticationListener):
    """
    The add delivery form that enables a user to place a delivery in the robot
    """

    def __init__(self, content):
        super(AddDelivery, self).__init__()
        self.content = content
        loadUi(os.path.join(UI_DIRECTORY, 'add_delivery.ui'), self)
        self.cancel_button.clicked.connect(self.back)
        self.save_button.clicked.connect(self.save)
        self.deliveries = {}
        self.sender = None
        AuthenticationClient.add_listener(self)

    def show(self, *args, **kwargs):
        super(AddDelivery, self).show()

        # Reset the combobox fields when this form is shown
        self.recipient_combobox.setCurrentIndex(0)
        self.location_combobox.setCurrentIndex(0)
        self.drawer_combobox.setCurrentIndex(0)

    def on_login(self, user):
        self.sender = user

    def on_login_failure(self, user):
        self.sender = None

    def on_logout(self, user):
        self.sender = None

    def save(self):
        self.content.components['open_drawer'].drawer_id = self.drawer_combobox.currentText()
        self.content.activate(self.content.components['open_drawer'])

    def back(self):
        self.content.activate(self.content.components['welcome'])
