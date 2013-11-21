__author__ = 'Rolf Jagerman'

from PySide import QtGui
import os

from loadui import loadUi
from config import UI_DIRECTORY, IMAGE_DIRECTORY
from authentication import AuthenticationClient, AuthenticationListener


class Error(QtGui.QFrame, AuthenticationListener):
    """
    The error message display
    """

    def __init__(self):
        super(Error, self).__init__()
        loadUi(os.path.join(UI_DIRECTORY, 'error.ui'), self)
        self.icon = QtGui.QImage()
        self.icon.load(os.path.join(IMAGE_DIRECTORY, 'error.png'))
        self.error_label.setPixmap(QtGui.QPixmap(self.icon))
        AuthenticationClient.add_listener(self)

    def on_login_failure(self, user):
        self.message_label.setText('Failed to login user')
