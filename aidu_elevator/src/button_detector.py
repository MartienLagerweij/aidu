#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import os
from time import clock
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


def threshold(image):
    if benchmarking:
        start = clock()

    # Pre process image
    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    image = cv2.medianBlur(image, 3)

    #image = cv2.Canny(image, 20, 100)

    # Apply adapative gaussian threshold
    image = cv2.adaptiveThreshold(image, 225, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 21, 3)

    #image = cv2.medianBlur(image, 3)

    if benchmarking:
        benchmark_threshold.append(clock() - start)

    # Return output
    return image


def detect_buttons(image):
    global n
    if benchmarking:
        start = clock()

    n += 1
    height, width = image.shape

    # Find all contours
    contours, hierarchy = cv2.findContours(image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    buttons = []

    # Iterate over contours, determining button on each contour
    for cnt in contours:
        if 450 < cv2.contourArea(cnt) < height * width * 0.9:  # remove very small and very large areas (noise)
            hull = cv2.convexHull(cnt) # find the convex hull of contour
            hull = cv2.approxPolyDP(hull, 0.1 * cv2.arcLength(hull, True), True)
            min_location = [width, height]
            max_location = [0.0, 0.0]
            if 4 <= len(hull) <= 6:
                margin = 3
                for point in hull:
                    x = point[0][0]
                    y = point[0][1]
                    if x < min_location[0]:
                        min_location[0] = max(1, x - margin)
                    if y < min_location[1]:
                        min_location[1] = max(1, y - margin)
                    if x > max_location[0]:
                        max_location[0] = min(width, x + margin)
                    if y > max_location[1]:
                        max_location[1] = min(height, y + margin)
                button = Button()
                button.x = 0.5*(max_location[0]+min_location[0])
                button.y = 0.5*(max_location[1]+min_location[1])
                button.w = max_location[0]-min_location[0]
                button.h = max_location[1]-min_location[1]
                if button.w > 0 and button.h > 0 and 0.9 < (button.w / button.h) < 1.1:
                    buttons.append(button)

    if benchmarking:
        benchmark_detect.append(clock() - start)

    return buttons


def test():
    db = MongoClient()
    fotos = db['aidu']['elevator_fotos']
    for idx, foto in enumerate(fotos.find()):
        rospy.loginfo("Processing foto %d" % idx)
        convert_start = clock()
        image = convert(foto['image'], input_type='ros', output_type='cv2')
        convert_elapsed = clock() - convert_start
        benchmark_convert.append(convert_elapsed)

        original_image = np.array(image)

        threshold_start = clock()
        image = threshold(image)
        threshold_elapsed = clock() - threshold_start
        benchmark_threshold.append(threshold_elapsed)

        threshold_image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

        detect_start = clock()
        buttons = detect_buttons(image)
        detect_elapsed = clock() - detect_start

        for button in buttons:
            x0 = int(button.x - button.w/2)
            y0 = int(button.y - button.h/2)
            x1 = int(button.x + button.w/2)
            y1 = int(button.y + button.h/2)
            cv2.rectangle(original_image, (x0, y0), (x1, y1), (255, 0, 0), 2)
            cv2.rectangle(threshold_image, (x0, y0), (x1, y1), (0, 0, 255), 2)
            #cv2.imshow('Button detector', cv2.resize(original_image[y0:y1, x0:x1], (300, 300)))
            #cv2.waitKey(1)

        rospy.loginfo('Foto %d - convert: %fs - threshold: %fs - detect: %fs' % (idx, convert_elapsed, threshold_elapsed, detect_elapsed))
        rospy.loginfo('Bytes: %d' % len(foto['image']))
        rospy.loginfo('Detected %d buttons' % len(buttons))

        out = np.vstack((original_image, threshold_image))
        cv2.imshow('Button detector', cv2.resize(out, (640, 720)))
        cv2.waitKey()

        if rospy.is_shutdown():
            break



def callback(image):
    global benchmarking, benchmark_convert, benchmark_detect, benchmark_threshold

    if benchmarking:
        convert_start = clock()

    image = convert(image.data, input_type='ros', output_type='cv2')

    if benchmarking:
        benchmark_convert.append(clock() - convert_start)

    image = threshold(image)
    buttons = detect_buttons(image)

    for button in buttons:
        x0, y0 = (int(button.x - button.w/2), int(button.y - button.h/2))
        x1, y1 = (int(button.x + button.w/2), int(button.y + button.h/2))
        button.image.data = convert(cv2.resize(image[y0:y1,x0:x1], (100, 100)), input_type='cv2', output_type='ros')
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
        cv2.namedWindow('Button detector')
        cv2.waitKey(1)
        test()
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
        rospy.loginfo('Image Conversion - Avg: %f sec - Std: %f sec' % (arr_convert.mean(), arr_convert.std()))
        rospy.loginfo('Image Threshold  - Avg: %f sec - Std: %f sec' % (arr_threshold.mean(), arr_threshold.std()))
        rospy.loginfo('Button Detection - Avg: %f sec - Std: %f sec' % (arr_detect.mean(), arr_detect.std()))



if __name__ == '__main__':
    main()
