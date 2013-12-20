import os
import scipy as sp
import numpy as np
import logging

import cv2

from timing import timing


# Set logging to correct level
logging.getLogger().setLevel(logging.INFO)

# Get relative path to images, independent of current location
data_path = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../data/'))
detection_directory = os.path.abspath(os.path.join(data_path, 'detection'))

# Get all file names in the directory, and initialize images and processed images
files = [file_name[2] for file_name in os.walk(detection_directory)][0]
images = []
processed_images = []


@timing
def get_image(file_name):
    return cv2.imread(os.path.join(detection_directory, file_name))


@timing
def process_image(image):
    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    image = cv2.medianBlur(image, 5)

    # Canny edge detection
    #output = cv2.Canny(image,100,200)

    # Adapative gaussian threshold
    output = cv2.adaptiveThreshold(image, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)

    return output


@timing
def detect_possible_buttons(image, source_image):
    contours, hierarchy = cv2.findContours(image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    buttons = []
    for cnt in contours:
        if 850 < cv2.contourArea(cnt) < 3000:  # remove small and large areas like noise etc
            hull = cv2.convexHull(cnt)    # find the convex hull of contour
            hull = cv2.approxPolyDP(hull, 0.1 * cv2.arcLength(hull, True), True)
            min = [10000.0, 10000.0]
            max = [0.0, 0.0]
            #print '%d,%d' % (point[0][0], point[0][1])
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
                points = [[min[0], min[1]], [max[0], min[1]], [max[0], max[1]], [min[0], max[1]]]
                points = np.array(points,np.int0)
                button = {'image': source_image[min[0]:max[0], min[1]:max[1]],
                          'x': 0.5*(max[0]+min[0]), 'y': 0.5*(max[1]+min[1]),
                          'w': max[0]-min[0], 'h': max[1]-min[1]}
                buttons.append(button)
                cv2.polylines(source_image, [points], True, (255, 0, 0), 2)
                cv2.drawContours(source_image, [hull], 0, (0, 255, 0), 1)
    return buttons


@timing
def get_all_images():
    global images, files
    images = [get_image(file_name) for file_name in files]


@timing
def process_all_images():
    global processed_images, images
    processed_images = [process_image(image) for image in images]
    contours = [detect_possible_buttons(image, images[idx]) for idx, image in enumerate(processed_images)]


def switch_image(image_index):
    try:
        img1 = cv2.resize(images[image_index], (360, 640))
        img2 = cv2.resize(processed_images[image_index], (360, 640))
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
    logging.info('Loading images...')
    get_all_images()

    logging.info('Processing all images...')
    process_all_images()

    logging.info('Constructing GUI...')
    size = len(images)
    cv2.namedWindow('image')
    cv2.createTrackbar('Selector', 'image', 0, size, switch_image)
    switch_image(0)

    logging.info('Done!')
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()