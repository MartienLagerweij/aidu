import cv2
import cv
import sys
import os
import scipy as sp
import numpy as np
import logging


def get_image(file_name):
    return cv2.imread(os.path.join(detection_directory, file_name))


def process_image(image):
    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    image = cv2.medianBlur(image,5)
    ret,th1 = cv2.threshold(image,127,255,cv2.THRESH_BINARY)
    th2 = cv2.adaptiveThreshold(image,255,cv2.ADAPTIVE_THRESH_MEAN_C,\
                cv2.THRESH_BINARY,11,2)
    th3 = cv2.adaptiveThreshold(image,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
                cv2.THRESH_BINARY,11,2)
    return th2

logging.getLogger().setLevel(logging.INFO)
data_path = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../data/'))
detection_directory = os.path.abspath(os.path.join(data_path, 'detection'))
files = [file_name[2] for file_name in os.walk(detection_directory)][0]
images = [get_image(file_name) for file_name in files]
processed_images = [process_image(image) for image in images]
output_image = images[0]


def switch_image(id):
    try:
        img1 = cv2.resize(images[id], (360,640))
        img2 = cv2.resize(processed_images[id], (360,640))
        h1, w1 = img1.shape[:2]
        h2, w2 = img2.shape[:2]
        view = sp.zeros((max(h1, h2), w1 + w2, 3), sp.uint8)
        view[:h1, :w1, :] = img1
        view[:h2, w1:, 0] = img2
        view[:h2, w1:, 1] = img2
        view[:h2, w1:, 2] = img2
        cv2.imshow('image', view)
    except IndexError:
        pass


def main():
    size = len(images)

    cv2.namedWindow('image')
    cv2.createTrackbar('Selector','image', 0, size, switch_image)
    switch_image(0)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()