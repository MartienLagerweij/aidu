__author__ = 'Rolf Jagerman'

from PySide import QtGui
import os

from loadui import loadUi
from config import UI_DIRECTORY, IMAGE_DIRECTORY


class ScanCard(QtGui.QFrame):
    """
    The idle display that shows a user that he/she should scan their campus card
    """

    def __init__(self):
        super(ScanCard, self).__init__()
        loadUi(os.path.join(UI_DIRECTORY, 'scan_card.ui'), self)
        self.animation = QtGui.QMovie(os.path.join(IMAGE_DIRECTORY, 'scan_card.gif'))
        self.image_label.setMovie(self.animation)
        self.animation.start()
