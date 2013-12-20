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
    height, width, depth = image.shape
    contours, hierarchy = cv2.findContours(processed_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    buttons = []
    for cnt in contours:
        if 850 < cv2.contourArea(cnt) < 3000:  # remove small and large areas like noise etc
            hull = cv2.convexHull(cnt)    # find the convex hull of contour
            hull = cv2.approxPolyDP(hull, 0.1 * cv2.arcLength(hull, True), True)
            min_location = [width, height]
            max_location = [0.0, 0.0]
            if len(hull) == 4:
                margin = 2
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
                if button.w > 0 and button.h > 0:
                    print '[%d:%d, %d:%d]' % (min_location[0], max_location[0], min_location[1], max_location[1])
                #    print image[min_location[0]:max_location[0], min_location[1]:max_location[1]]
                    button.image = CompressedImage()
                    button.image.header.stamp = rospy.Time.now()
                    button.image.header.frame_id = "camera"
                    button.image.format = 'rgb8; jpeg compressed bgr8'
                    subregion = image[min_location[1]:max_location[1], min_location[0]:max_location[0], :]
                    print subregion
                    some_image = cv2.imencode('.jpg', subregion[0])[1]
                    button.image.data = np.array(some_image).tostring()
                    image_publisher.publish(button.image)
                    #buttons.append(button)

                points = [[min_location[0], min_location[1]], [max_location[0], min_location[1]], [max_location[0], max_location[1]], [min_location[0], max_location[1]]]
                points = np.array(points,np.int0)
                cv2.polylines(image, [points], True, (255, 0, 0), 2)
                cv2.drawContours(image, [hull], 0, (0, 255, 0), 1)
    return buttons


def callback(image):
    image_publisher.publish(image)
    #np_arr = np.fromstring(image.data, np.uint8)
    #image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
    #for button in detect_buttons(image_np):
    #    button_publisher.publish(button)


def main():
    global button_publisher, image_publisher
    rospy.init_node('button_detector', anonymous=True)
    rospy.Subscriber("/image_raw/compressed", CompressedImage, callback)
    image_publisher = rospy.Publisher("/elevator/button/image/compressed", CompressedImage)
    button_publisher = rospy.Publisher("/elevator/button", Button)
    rospy.spin()


if __name__ == '__main__':
    main()
