#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from aidu_elevator.msg import Button


def process_image(image):
    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    image = cv2.medianBlur(image, 5)

    # Canny edge detection
    #output = cv2.Canny(image,100,200)

    # Adapative gaussian threshold
    output = cv2.adaptiveThreshold(image, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
    return output


def detect_buttons(image):
    processed_image = process_image(image)
    contours, hierarchy = cv2.findContours(processed_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    buttons = []
    for cnt in contours:
        if 850 < cv2.contourArea(cnt) < 3000:  # remove small and large areas like noise etc
            hull = cv2.convexHull(cnt)    # find the convex hull of contour
            hull = cv2.approxPolyDP(hull, 0.1 * cv2.arcLength(hull, True), True)
            min = [10000.0, 10000.0]
            max = [0.0, 0.0]
            if len(hull) == 4:
                margin = 2
                for point in hull:
                    x = point[0][0]
                    y = point[0][1]
                    if x < min[0]:
                        min[0] = x - margin
                    if y < min[1]:
                        min[1] = y - margin
                    if x > max[0]:
                        max[0] = x + margin
                    if y > max[1]:
                        max[1] = y + margin
                button = Button()
                button.x = 0.5*(max[0]+min[0])
                button.y = 0.5*(max[1]+min[1])
                button.w = max[0]-min[0]
                button.h = max[1]-min[1]
                button.image = CompressedImage()
                button.image.header.stamp = rospy.Time.now()
                button.image.format = "jpeg"
                button.image.data = np.array(cv2.imencode('.jpg', image[min[0]:max[0], min[1]:max[1]])[1]).tostring()
                buttons.append(button)

                #points = [[min[0], min[1]], [max[0], min[1]], [max[0], max[1]], [min[0], max[1]]]
                #points = np.array(points,np.int0)
                #cv2.polylines(image, [points], True, (255, 0, 0), 2)
                #cv2.drawContours(image, [hull], 0, (0, 255, 0), 1)
    return buttons


def process_image(image):
    np_arr = np.fromstring(image.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
    for button in detect_buttons(image_np):
        button_publisher.publish(button)


def main():
    global button_publisher
    rospy.init_node('button_detector', anonymous=True)
    rospy.Subscriber("/webcam/image/compressed", CompressedImage, process_image)
    button_publisher = rospy.Publisher("/elevator/button", Button)
    rospy.spin()


if __name__ == '__main__':
    main()
