__author__ = 'Rolf Jagerman'

import rospy
from aidu_user_management.srv import *
from gui_thread import invoke_in_gui_thread


class AuthenticationClient:

    listeners = []

    def __init__(self):
        pass

    @staticmethod
    def login_handler(authentication):
        invoke_in_gui_thread(AuthenticationClient.process_authentication, authentication)

    @staticmethod
    def process_authentication(authentication):
        for listener in AuthenticationClient.listeners:
            listener.on_authenticate(authentication)

    @staticmethod
    def add_listener(authentication_listener):
        AuthenticationClient.listeners.append(authentication_listener)


class AuthenticationListener(object):
    """
    Abstract class for GUI elements to listen to authentication changes
    """

    def __init__(self):
        pass

    def on_authenticate(self, authentication):
        if authentication.login:
            if authentication.success:
                self.on_login(authentication.user)
            else:
                self.on_login_failure(authentication.user)
        else:
            self.on_logout(authentication.user)

    def on_login(self, user):
        pass

    def on_logout(self, user):
        pass

    def on_login_failure(self, user):
        pass