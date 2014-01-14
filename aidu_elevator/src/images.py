__author__ = 'rolf'

import cv2
import numpy as np


def convert(image, input_type='cv2', output_type='ros'):
    assert input_type in ['cv2', 'ros', 'mongo'] and output_type in ['cv2', 'ros', 'mongo']
    if input_type == 'mongo':
        image = cv2.imdecode(np.array(image, dtype=np.uint8), 1)
    if input_type == 'ros':
        image = cv2.imdecode(np.fromstring(image, np.uint8), cv2.CV_LOAD_IMAGE_COLOR)
    if output_type == 'ros':
        return np.array(cv2.imencode('.jpg', image)[1]).tostring()
    if output_type == 'mongo':
        return np.array(cv2.imencode('.jpg', image)[1]).tolist()
    if output_type == 'cv2':
        return image