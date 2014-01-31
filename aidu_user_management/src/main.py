#!/usr/bin/env python
__author__ = 'Rolf Jagerman'

import roslib; roslib.load_manifest('aidu_user_management')
import rospy
from aidu_user_management.srv import Authenticate, AuthenticateResponse
from aidu_user_management.msg import User, Authentication

authenticate_service = None
authentication_publisher = None


def authenticate(req):
    # TODO: Get real user from database
    rospy.loginfo("Processing authentication request");
    success = False
    user = None
    if req.id == 'campus-card-code':
        success = True
        user = User(req.id, 'Firstname', 'Lastname', [1])
    #if req.id == 'campus-card-code':
    #    success = True
    #    user = User(req.id, 'Firstname', 'Lastname', [1])
    authentication_publisher.publish(Authentication(req.login, success, user))
    return AuthenticateResponse(req.id, success, user)

if __name__ == "__main__":
    rospy.init_node('user_management')
    authenticate_service = rospy.Service('authenticate', Authenticate, authenticate)
    authentication_publisher = rospy.Publisher('authentication', Authentication)
    rospy.spin()

