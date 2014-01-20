#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import os
from datetime import datetime
from time import clock, time
from sensor_msgs.msg import CompressedImage
from aidu_elevator.msg import Button
from images import convert
from pymongo import MongoClient

data_path = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../data/'))
detection_directory = os.path.abspath(os.path.join(data_path, 'detection'))

n = 0
benchmarking = False
benchmark_convert = []
benchmark_detect = []
benchmark_threshold = []


def millis(dt):
    """
    Converts a datetime object to milliseconds
    """
    return (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0


def threshold(image):
    """
    Thresholds an image
    """
    global n
    n += 1
    if benchmarking:
        start = datetime.now()

    # Pre process image
    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    #image = cv2.equalizeHist(image)

    m = 1 + n / 2
    #adapt_gaus = cv2.adaptiveThreshold(image, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 75, 7)
    #cv2.imwrite('/home/rolf/Desktop/Assignment 6/thresholding/%s_%d.jpg' % ("adaptive_gaussian", m), adapt_gaus)

    adapt_mean = cv2.adaptiveThreshold(image, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 75, 7)
    #cv2.imwrite('/home/rolf/Desktop/Assignment 6/thresholding/%s_%d.jpg' % ("adaptive_mean", m), adapt_mean)
    #
    #reg_binary = cv2.threshold(image, 128, 255, cv2.THRESH_BINARY)[1]
    #cv2.imwrite('/home/rolf/Desktop/Assignment 6/thresholding/%s_%d.jpg' % ("binary", m), reg_binary)
    #
    #reg_tozero = cv2.threshold(image, 128, 255, cv2.THRESH_TOZERO)[1]
    #cv2.imwrite('/home/rolf/Desktop/Assignment 6/thresholding/%s_%d.jpg' % ("to_zero", m), reg_tozero)

    image = adapt_mean

    if benchmarking:
        benchmark_threshold.append(millis(datetime.now() - start))

    # Return output
    return image


def detect_buttons(image, min_button_size=0.002, max_button_size=0.8, max_stretch=0.08):
    """
    Attempts to detect buttons in given image
    """
    global n
    if benchmarking:
        start = datetime.now()

    n += 1
    height, width = image.shape

    # Find all contours
    contours, hierarchy = cv2.findContours(image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    buttons = []

    # Iterate over contours, determining if there's a button on each contour
    for cnt in contours:
        if min_button_size * width * height < cv2.contourArea(cnt) < max_button_size * height * width:  # remove very small and very large areas (noise)
            hull = cv2.convexHull(cnt)
            hull = cv2.approxPolyDP(hull, 0.1 * cv2.arcLength(hull, True), True)
            if 4 <= len(hull) <= 6:
                x,y,w,h = cv2.boundingRect(cnt)
                button = Button()
                button.x = x + w/2; button.y = y + h/2; button.w = w; button.h = h;
                if button.w > 0 and button.h > 0 and (1.0 - max_stretch) < (float(button.w) / button.h) < (1.0 + max_stretch):
                    buttons.append(button)

    if benchmarking:
        benchmark_detect.append(millis(datetime.now() - start))

    return buttons


def test():
    """
    Tests the button detection algorithm on our dataset
    """
    db = MongoClient()
    fotos = db['aidu']['elevator_fotos']
    for idx, foto in enumerate(fotos.find()):
        #if idx not in [143, 196, 206, 524]:
        #    continue

        rospy.loginfo("Processing foto %d" % idx)
        convert_start = datetime.now()
        image = convert(foto['image'], input_type='ros', output_type='cv2')
        benchmark_convert.append(millis(datetime.now() - convert_start))

        original_image = np.array(image)

        # Threshold
        image = threshold(image)
        threshold_image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

        # Detect buttons
        buttons = detect_buttons(image)

        rospy.loginfo('Foto %d - convert: %fms - threshold: %fms - detect: %fms' % (idx, benchmark_convert[-1],
                                                                                    benchmark_threshold[-1],
                                                                                    benchmark_detect[-1]))
        rospy.loginfo('Bytes: %d' % len(foto['image']))
        rospy.loginfo('Detected %d buttons' % len(buttons))

        # Show button
        for button in buttons:
            x0 = int(button.x - button.w/2)
            y0 = int(button.y - button.h/2)
            x1 = int(button.x + button.w/2)
            y1 = int(button.y + button.h/2)
            cv2.rectangle(original_image, (x0, y0), (x1, y1), (0, 0, 255), 2)
            cv2.rectangle(threshold_image, (x0, y0), (x1, y1), (0, 0, 255), 2)
        out = np.vstack((original_image, threshold_image))
        cv2.imshow('Button detector', cv2.resize(out, (640, 720)))
        cv2.waitKey()

        if rospy.is_shutdown():
            break


def callback(image):
    global benchmarking, benchmark_convert, benchmark_detect, benchmark_threshold

    if benchmarking:
        convert_start = datetime.now()

    image = convert(image.data, input_type='ros', output_type='cv2')

    if benchmarking:
        benchmark_convert.append(millis(datetime.now() - convert_start))

    image = threshold(image)
    buttons = detect_buttons(image)

    for button in buttons:
        x0 = int(button.x - button.w/2)
        y0 = int(button.y - button.h/2)
        x1 = int(button.x + button.w/2)
        y1 = int(button.y + button.h/2)
        button.image.data = convert(cv2.resize(image[y0:y1,x0:x1], (100, 100)), input_type='cv2', output_type='ros')
        image_publisher.publish(button.image)
        button_publisher.publish(button)

    rospy.loginfo('Detected %d buttons' % len(buttons))

    #image_publisher.publish(image)
    #mongo_image = convert(image.data, input_type='ros', output_type='mongo')
    #cv2_image = convert(image.data, input_type='ros', output_type='cv2')
    #cv2_image = cv2.resize(cv2_image, (1280/2, 720/2))
    #cv2.imshow('t', cv2_image)
    #cv2.waitKey(0)
    #fotos.insert({'foto': mongo_image})


def main():
    global benchmarking, benchmark_convert, benchmark_detect, benchmark_threshold, button_publisher, image_publisher

    rospy.init_node('button_detector', anonymous=True)

    if rospy.has_param('~benchmark'):
        benchmarking = True

    if rospy.has_param('~test'):
        test()
        cv2.destroyAllWindows()
    else:
        rospy.Subscriber("/image_raw/compressed", CompressedImage, callback)
        image_publisher = rospy.Publisher("/elevator/button/image/compressed", CompressedImage)
        button_publisher = rospy.Publisher("/elevator/button", Button)

    rospy.spin()

    if benchmarking:
        arr_convert = np.array(benchmark_convert)
        arr_detect = np.array(benchmark_detect)
        arr_threshold = np.array(benchmark_threshold)
        rospy.loginfo('Number of images processed: %d' % n)
        rospy.loginfo('Image Conversion - Avg: %f ms - Std: %f ms' % (arr_convert.mean(), arr_convert.std()))
        rospy.loginfo('Image Threshold  - Avg: %f ms - Std: %f ms' % (arr_threshold.mean(), arr_threshold.std()))
        rospy.loginfo('Button Detection - Avg: %f ms - Std: %f ms' % (arr_detect.mean(), arr_detect.std()))



if __name__ == '__main__':
    main()
