__author__ = 'rolf'

import os
import cv2


data_path = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../data/'))
detection_directory = os.path.abspath(os.path.join(data_path, 'detection'))

for file in os.listdir(detection_directory):
    img = cv2.imread('%s/%s' % (detection_directory, file))
    print cv2.imencode('.jpg', img)
