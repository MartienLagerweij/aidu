import cv2
import os
import sys
import numpy as np
import scipy as sp
import pylab as pl
from datetime import datetime
from sklearn.linear_model import LogisticRegression
from sklearn.metrics import confusion_matrix, classification_report
from button_classifier import train_save_test
from matplotlib import pyplot as plt
from pymongo import MongoClient
from bson.binary import Binary
from images import convert
from time import clock
from math import floor


data_path = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../data/'))
match_directory = os.path.abspath(os.path.join(data_path, 'matching'))
db = MongoClient()
buttons = db['aidu']['elevator_buttons']
query_images = {}
benchmarking = False
benchmark_detector = []


def millis(dt):
    """
    Converts a datetime object to milliseconds
    """
    return (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0


def preprocess(image):
    """
    Preprocesses an image by applying a thresholding
    """
    return cv2.cvtColor(
        cv2.medianBlur(
            cv2.adaptiveThreshold(
                cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        ,  255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 31, 3
            ), 5
        )\
        ,cv2.COLOR_GRAY2BGR
    )


def cut_to_bounding_box(image):
    """
    Cuts an image to the largest bounding box that it can find
    """
    img = image
    contours, hierarchy = cv2.findContours(cv2.cvtColor(255 - img, cv2.COLOR_BGR2GRAY), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    maxArea = 0
    x,y,w,h = (0, 0, 100, 100)
    for cnt in contours:
        if cv2.contourArea(cnt) > maxArea:
            x,y,w,h = cv2.boundingRect(cnt)
            maxArea = cv2.contourArea(cnt)
    return img[y:y+h, x:x+w]


def drawMatches(queryImage, trainImage, k1, k2, matches):
    """
    Draws the matches that were detected, for debugging purposes
    """
    img1 = queryImage
    img2 = trainImage

    h1, w1 = img1.shape[:2]
    h2, w2 = img2.shape[:2]
    view = sp.zeros((max(h1, h2), w1 + w2, 3), sp.uint8)
    view[:h1, :w1, :] = img1
    view[:h2, w1:, :] = img2
    view[:, :, 1] = view[:, :, 0]
    view[:, :, 2] = view[:, :, 0]

    for m in matches:
        color = tuple([sp.random.randint(0, 255) for _ in xrange(3)])
        cv2.line(view, (int(k1[m.queryIdx].pt[0]), int(k1[m.queryIdx].pt[1])), (int(k2[m.trainIdx].pt[0] + w1), int(k2[m.trainIdx].pt[1])), color)

    cv2.imshow("view", view)
    cv2.waitKey()


def keypoint_match(queryImage, trainImage):
    img1 = queryImage # queryImage
    img2 = trainImage # trainImage

    # Initiate SIFT detector
    sift = cv2.SIFT()
    sift.compute()
    # find the keypoints and descriptors with SIFT
    kp1, des1 = sift.detectAndCompute(img1,None)
    kp2, des2 = sift.detectAndCompute(img2,None)

    # BFMatcher with default params
    #kp3, des3 = cv2.goodFeaturesToTrack(cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY), 12, 3, 1)
    bf = cv2.BFMatcher()
    matches = bf.knnMatch(des1, des2, k=2)

    # Apply ratio test
    good = []
    for m,n in matches:
        if m.distance < 0.95*n.distance:
            good.append(m)

    # Score
    score = 0
    for m in good:
        score += 1 / (m.distance + 1)

    # cv2.drawMatchesKnn expects list of lists as matches.
    #drawMatches(queryImage, trainImage, kp1, kp2, good)
    return score


def detect_keypoints(image, detector=''):

    feature_detector = cv2.FeatureDetector_create(detector)
    if benchmarking:
        start = clock()

    kpts = feature_detector.detect(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY))

    if benchmarking:
        benchmark_detector.append(clock() - start)

    return kpts


def compute_descriptors(image, keypoints, descriptor=''):
    pass


def match(image, trainImage, matcher):
    pass


def get_query_images():
    return {f[:-4]: preprocess(cv2.imread(os.path.join(match_directory, f))) for f in os.listdir(match_directory) if '-' not in f}


def progressor(generator):
    for idx, item in enumerate(generator):
        sys.stdout.write('\r[%d samples loaded]' % idx)
        sys.stdout.flush()
        yield item
    sys.stdout.write('\n')


def test_results(y, y_res, lbl, algo = ''):
    cm = confusion_matrix(y, y_res, labels=lbl)
    cm = cm / cm.astype(np.float).sum(axis=1)
    print classification_report(y, y_res, labels=lbl)

    # Show confusion matrix in a separate window
    pl.matshow(cm)
    pl.title('Confusion matrix')
    pl.colorbar()
    pl.ylabel('True label')
    pl.xlabel('Predicted label')
    pl.savefig('/home/rolf/Desktop/Assignment 6/plots/matching/%s.pdf' % algo)


def test_technique(X, y, labels, detector_name='', descriptor_name='', matcher_name=''):
    sys.stdout.write('Testing - Detector:%s - Descriptor:%s - Matcher:%s' % (detector_name, descriptor_name, matcher_name))
    sys.stdout.flush()

    start = clock()
    y_res = []
    y_real = []
    Xt = []
    benchmark = []

    detector = cv2.FeatureDetector_create(detector_name)
    descriptor = cv2.DescriptorExtractor_create(descriptor_name)
    matcher = cv2.DescriptorMatcher_create(matcher_name)
    k = 6

    # Compute query image descriptors
    query_images_descriptors = {}
    query_images_keypoints = {}
    for key, query_image in query_images.iteritems():
        query_images_keypoints[key] = detector.detect(query_image)
        query_images_descriptors[key] = descriptor.compute(query_image, query_images_keypoints[key])[1]
        if query_images_descriptors[key] is not None:
            query_images_descriptors[key] = query_images_descriptors[key].astype('float32')

    # Compute matches for every image in dataset X
    scores = {x: [] for x in query_images}
    sys.stdout.write(' ' * 10)
    for idx, image in enumerate(X):
        sys.stdout.write('\b' * 10 + ('%d' % idx).rjust(10))
        sys.stdout.flush()
        keypoints = detector.detect(image)
        if len(keypoints) > 0:
            descriptors = descriptor.compute(image, keypoints)[1]
            if descriptors is not None:
                descriptors = descriptors.astype('float32')
                current_k = max(1, min(k, len(descriptors)-1))
                max_score = 0
                label = 'NONE'
                #print ''
                #print y[idx]
                start_time = datetime.now()
                for key, query_image_descriptors in query_images_descriptors.iteritems():
                    if query_image_descriptors is not None:
                        try:
                            image_matches = matcher.match(query_image_descriptors, trainDescriptors = descriptors)#, k = current_k)
                            score = 1.0
                            good_matches = []
                            for m in image_matches:
                                good_matches.append((m.distance, m))
                            good_matches.sort(key=lambda tup: tup[0])
                            good_matches = [m for d, m in good_matches[:10]]
                            #print key, ' - ', [m.distance for m in good_matches][:4]
                            for m in good_matches:
                                score += m.distance
                            score = 1.0 / score
                            if score > max_score:
                                max_score = score
                                label = key
                            if key == y[idx]:
                                scores[key].append(score)
                        except Exception as e:
                            print e
                            print descriptors
                            print len(descriptors[0])
                            print len(descriptors[1])
                            raise Exception("Quit")
                #print label, ' - ', max_score
                y_res.append((label, max_score))
                y_real.append(y[idx])
                benchmark.append(millis(datetime.now() - start_time))
    print('')

    for key, key_scores in scores.iteritems():
        npa = np.array(key_scores)
        #print np.median(npa), ' - ', npa.mean(), ' - ', npa.std()
        try:
            scores[key] = np.percentile(npa, 60) #- npa.std()/10
        except:
            scores[key] = npa.mean()

    for idx, (label, score) in enumerate(y_res):
        if score > scores[key]:
            y_res[idx] = label
        else:
            y_res[idx] = 'NONE'
    # Test the found results
    #clf = LogisticRegression(class_weight='auto')
    #clf.fit(Xt, y_real)
    #y_pred = clf.predict(Xt)

    npa = np.array(benchmark)
    print 'Benchmark - Mean: %.2fms - Std: %.2fms' % (npa.mean(), npa.std())
    print 'Benchmark - Total: %d minutes %.2f seconds' % (int(floor((clock() - start) / 60)), (clock() - start) % 60)
    test_results(y_real, y_res, labels, algo='%s_%s_%s' % (detector_name, descriptor_name, matcher_name))


def test():
    """
    Runs the test set with all possible detectors, descriptors and matchers,
    """

    # Get the query images
    global query_images
    query_images = get_query_images()
    for key, image in query_images.iteritems():
        query_images[key][ 0: 18,  0:100] = [255, 255, 255]
        query_images[key][82:100,  0:100] = [255, 255, 255]
        query_images[key][ 0:100,  0: 18] = [255, 255, 255]
        query_images[key][ 0:100, 82:100] = [255, 255, 255]

    # Get the data test set
    X = []
    y = []
    labels = []
    for button in progressor(buttons.find(snapshot=True)):
        try:
            image = preprocess(convert(button['image'], input_type='ros', output_type='cv2'))
            label = button['label'].upper() if button['label'] is not None else 'NONE'
            if label not in labels:
                labels.append(label)
            X.append(image)
            y.append(label)
        except:
            print 'Skipping sample'

    # Set up which detectors, descriptors and matchers to use for testing
    detector_formats = [""]#,"Grid","Pyramid"]
    detector_types = ["HARRIS", "SIFT", "SURF", "ORB", "MSER", "GFTT"]
    descriptor_types = ["SIFT", "SURF", "ORB"]
    matcher_types = ["BruteForce", "FlannBased"]

    # Test all combinations
    for detector_format in detector_formats:
        for detector_type in detector_types:
            for descriptor_type in descriptor_types:
                for matcher_type in matcher_types:
                    if descriptor_type == 'ORB' and detector_type == 'SIFT':
                        continue
                    test_technique(X, y, labels, detector_name=detector_format + detector_type, descriptor_name=descriptor_type,
                                   matcher_name=matcher_type)


def main():
    test()

if __name__ == "__main__":
    main()

#max_score = 0
#label = cls = 'NONE'
#x = np.zeros(len(query_images))
#for idx, (key, qimage) in enumerate(query_images.iteritems()):
#    score = keypoint_match(qimage, image)
#    x[idx] = score
#    if score > max_score:
#        max_score = score
#        cls = key
#if max_score > 0.025:
#    label = cls
#X.append(x)
#y.append(button['label'].upper() if button['label'] is not None else 'NONE')
#y_res.append(label.upper())
#if y[-1] not in labels:
#    labels.append(y[-1])