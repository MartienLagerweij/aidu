#!/usr/bin/env python
__author__ = 'Rolf Jagerman'

from PySide import QtGui
import os

from loadui import loadUi
from config import UI_DIRECTORY


class AddDelivery(QtGui.QFrame):
    """
    The add delivery form that enables a user to place a delivery in the robot
    """

    def __init__(self):
        super(AddDelivery, self).__init__()
        loadUi(os.path.join(UI_DIRECTORY, 'add_delivery.ui'), self)

    def show(self, *args, **kwargs):
        super(AddDelivery, self).show()

        # Reset the combobox fields when this form is shown
        self.recipient_combobox.setCurrentIndex(0)
        self.location_combobox.setCurrentIndex(0)
        self.drawer_combobox.setCurrentIndex(0)
