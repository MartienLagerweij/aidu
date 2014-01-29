__author__ = 'Rolf Jagerman'

from PySide import QtGui, QtCore
from add_delivery import AddDelivery
from retrieve_delivery import RetrieveDelivery
from scan_card import ScanCard
from welcome import Welcome
from header import Header
from error import Error
from open_drawer import OpenDrawer
from authentication import AuthenticationClient, AuthenticationListener


class Window(QtGui.QFrame):
    """
    The main full screen window of the application. This contains the header and content sections. It also handles some
    program-specific styling.
    """

    def __init__(self):
        super(Window, self).__init__()
        self.setWindowFlags(QtCore.Qt.FramelessWindowHint)

        self.header = Header()
        self.content = Content()
        self.content.activate(self.content.components['scan_card'])

        vertical_layout = QtGui.QVBoxLayout()
        vertical_layout.addWidget(self.header)
        vertical_layout.addWidget(self.content)

        self.setLayout(vertical_layout)
        self.setStyleSheet("QFrame { background-color: #ffffff; }")


class Content(QtGui.QFrame, AuthenticationListener):
    """
    The main content area of the application. This will hold all possible dialog frames and can display one of them
    whenever activate() is called.
    """

    def __init__(self):
        super(Content, self).__init__()
        self.setFrameStyle(QtGui.QFrame.StyledPanel | QtGui.QFrame.Plain)
        self.timer = None

        # Initialize all possible content components
        self.components = {'scan_card': ScanCard(),
                           'add_delivery': AddDelivery(self),
                           'retrieve_delivery': RetrieveDelivery(self),
                           'welcome': Welcome(self),
                           'open_drawer': OpenDrawer(self),
                           'error': Error()}

        # Creates the layout and adds all the components
        layout = QtGui.QHBoxLayout()
        for component in self.components.values():
            layout.addWidget(component)
            component.hide()
        self.setLayout(layout)

        # Listen to the authentication client
        AuthenticationClient.add_listener(self)

    def activate(self, component):
        """
        Activates a content component and hides all the others
        """
        if self.timer is not None:
            self.timer.stop()
        for c in self.components.values():
            if c is not component:
                c.hide()
        component.show()

    def activate_after(self, component, time = 1000):
        """
        Activates a component only after given specified amount of time
        """
        if self.timer is not None:
            self.timer.stop()
        self.timer = QtCore.QTimer()
        self.connect(self.timer, QtCore.SIGNAL("timeout()"), lambda: self.activate(component))
        self.timer.setSingleShot(True)
        self.timer.start(time)

    def on_login(self, user):
        self.activate(self.components['welcome'])

    def on_login_failure(self, user):
        self.activate(self.components['error'])
        self.activate_after(self.components['scan_card'], 2500)

    def on_logout(self, user):
        self.activate(self.components['scan_card'])
