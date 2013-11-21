__author__ = 'Rolf Jagerman'

from PySide import QtGui
import os

from loadui import loadUi
from config import UI_DIRECTORY, IMAGE_DIRECTORY
from authentication import AuthenticationClient, AuthenticationListener


class Header(QtGui.QFrame, AuthenticationListener):
    """
    The header area of the application. Contains the logo of the software and the current logged in user
    """

    def __init__(self):
        super(Header, self).__init__()
        loadUi(os.path.join(UI_DIRECTORY, 'header.ui'), self)
        self.logo = QtGui.QImage()
        self.logo.load(os.path.join(IMAGE_DIRECTORY, 'aidu.png'))
        self.logo_label.setPixmap(QtGui.QPixmap(self.logo.scaledToHeight(100)))
        self.logout_button.clicked.connect(self.logout)
        AuthenticationClient.add_listener(self)

    def on_login(self, user):
        self.user_label.setText("Logged in as %s %s" % (user.first_name, user.last_name))
        self.logout_widget.show()

    def on_login_failure(self, user):
        self.on_logout(user)

    def on_logout(self, user):
        self.user_label.setText("Not logged in")
        self.logout_widget.hide()

    def logout(self):
        #AuthenticationClient.logout()
        pass

