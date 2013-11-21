__author__ = 'Rolf Jagerman'

import roslib; roslib.load_manifest('aidu_gui')
from PySide import QtGui, QtCore
from PySide.QtGui import QApplication
from time import sleep
from window import Window
from ros_thread import ROSThread


class Manager:
    """
    The manager for the application GUI. This internally handles all the other GUI elements that are necessary to
    display the system. It also handles shutdown events and appropriately shuts down the ROS thread whenever the user
    wants to exit the application.
    """

    def __init__(self):
        pass

    @staticmethod
    def setup():
        Manager.app = QApplication([])
        Manager.app.aboutToQuit.connect(Manager.exit)

        font = Manager.app.font()
        font.setPointSize(18)
        Manager.app.setFont(font)

        Manager.window = Window()
        Manager.window.showFullScreen()
        Manager.window.activateWindow()

        Manager.ros_thread = ROSThread(Manager.app)
        Manager.ros_thread.start()

        QtGui.QShortcut(QtGui.QKeySequence(QtCore.Qt.CTRL + QtCore.Qt.Key_Q), Manager.window,
                        Manager.window.close)

    @staticmethod
    def execute():
        """
        Starts execution of the GUI. Returns the application's exit code when it is shut down.
        """
        return Manager.app.exec_()

    @staticmethod
    def exit():
        """
        Callback function for when the user exits the application.
        This will attempt to stop the ROS thread and will wait before shutting the GUI down.
        """
        Manager.ros_thread.stop()
        while not Manager.ros_thread.done:
            try:
                sleep(0.1)
            except KeyboardInterrupt:
                break

