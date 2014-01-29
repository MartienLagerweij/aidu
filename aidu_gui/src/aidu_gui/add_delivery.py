#!/usr/bin/env python
__author__ = 'Rolf Jagerman'

from PySide import QtGui
import os
from authentication import AuthenticationListener, AuthenticationClient
from loadui import loadUi
from config import UI_DIRECTORY
from drawers import Drawers
from users import User
from locations import Location


class AddDelivery(QtGui.QFrame, AuthenticationListener):
    """
    The add delivery form that enables a user to place a delivery in the robot
    """

    def __init__(self, content):
        super(AddDelivery, self).__init__()

        self.content = content
        loadUi(os.path.join(UI_DIRECTORY, 'add_delivery.ui'), self)
        self.cancel_button.clicked.connect(self.cancel)
        self.save_button.clicked.connect(self.place)
        self.deliveries = {}
        self.sender = None
        self.drawer_id = '1'

        for location in Location.get_locations():
            self.location_combobox.addItem(location.name)

        #from PySide.QtGui import QComboBox
        #test = QComboBox()
        #self.recipient_combobox.addItems(User.get_users())
        #self.location_combobox.addItems(Location.get_locations())

        AuthenticationClient.add_listener(self)

    def show(self, *args, **kwargs):
        super(AddDelivery, self).show()
        self.prepare_open_drawer()

        while self.recipient_combobox.count() > 0:
            self.recipient_combobox.removeItem(0);

        for user in User.get_users():
            if user.id != self.sender.id:
                self.recipient_combobox.addItem(user.name)

        # Reset the combobox fields when this form is shown
        #from PySide.QtGui import QComboBox
        ##test = QComboBox()
        while self.drawer_combobox.count() > 0:
            self.drawer_combobox.removeItem(0)
        #self.drawer_combobox.removeItems()
        for drawer in Drawers.available_drawers():
            self.drawer_combobox.addItem(drawer)
        self.recipient_combobox.setCurrentIndex(0)
        self.location_combobox.setCurrentIndex(0)
        self.drawer_combobox.setCurrentIndex(0)

    def prepare_open_drawer(self):
        self.content.components['open_drawer'].save = lambda : self.save()
        self.content.components['open_drawer'].back = lambda : self.back()
        self.content.components['open_drawer'].success_message = 'Have you succesfully placed your delivery?'
        self.content.components['open_drawer'].instruction_message = 'place'


    def on_login(self, user):
        self.sender = user

    def on_login_failure(self, user):
        self.sender = None

    def on_logout(self, user):
        self.sender = None

    def place(self):
        self.drawer_id = self.drawer_combobox.currentText()
        recipient_text = self.recipient_combobox.currentText()
        for user in User.get_users():
            if user.name == recipient_text:
                self.recipient_id = user.id

        location_text = self.location_combobox.currentText()
        for location in Location.get_locations():
            if location.name == location_text:
                self.location_id = location.id

        self.content.components['open_drawer'].drawer_id = self.drawer_combobox.currentText()
        self.content.activate(self.content.components['open_drawer'])

    def cancel(self):
        self.content.activate(self.content.components['welcome'])

    def back(self):
        self.content.activate(self.content.components['add_delivery'])

    def save(self):
        Drawers.add_delivery(self.drawer_id, self.recipient_id, self.location_id)
        self.content.activate(self.content.components['welcome'])