import cv2
import os
import sys
import numpy as np
import scipy as sp
import pylab as pl
from sklearn.linear_model import LogisticRegression
from sklearn.metrics import confusion_matrix, classification_report
from button_classifier import train_save_test
from matplotlib import pyplot as plt
from pymongo import MongoClient
from bson.binary import Binary
from images import convert


data_path = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../data/'))
match_directory = os.path.abspath(os.path.join(data_path, 'matching'))
db = MongoClient()
buttons = db['aidu']['elevator_buttons']


def preprocess(image):

    return cv2.cvtColor(
        cv2.medianBlur(
            cv2.adaptiveThreshold(
                cv2.cvtColor(image, cv2.COLOR_BGR2GRAY),  255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 21, 3
            ), 5
        ), cv2.COLOR_GRAY2BGR
    )


def cut_to_bounding_box(image):
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
    img1 = queryImage
    img2 = trainImage

    # #####################################
    # visualization
    h1, w1 = img1.shape[:2]
    h2, w2 = img2.shape[:2]
    view = sp.zeros((max(h1, h2), w1 + w2, 3), sp.uint8)
    view[:h1, :w1, :] = img1
    view[:h2, w1:, :] = img2
    view[:, :, 1] = view[:, :, 0]
    view[:, :, 2] = view[:, :, 0]

    for m in matches:
        # draw the keypoints
        # print m.queryIdx, m.trainIdx, m.distance
        color = tuple([sp.random.randint(0, 255) for _ in xrange(3)])
        cv2.line(view, (int(k1[m.queryIdx].pt[0]), int(k1[m.queryIdx].pt[1])), (int(k2[m.trainIdx].pt[0] + w1), int(k2[m.trainIdx].pt[1])), color)

    cv2.imshow("view", view)
    cv2.waitKey()


def keypoint_match(queryImage, trainImage):


    img1 = queryImage # queryImage
    img2 = trainImage # trainImage

    # Initiate SIFT detector
    sift = cv2.SIFT()

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


def template_match(queryImage, trainImage):
    pass
    #img = cv2.resize(queryImage[y:y+h, x:x+w], (50, 50))



def get_query_images():
    return {f[:-4]: preprocess(cv2.imread(os.path.join(match_directory, f))) for f in os.listdir(match_directory) if '-' not in f}
    #img = cv2.imread(os.path.join(match_directory, f))
    #cv2.imwrite(os.path.join(match_directory, f), img[28:128,1:101])


def progressor(generator):
    for idx, item in enumerate(generator):
        sys.stdout.write('\r[%d samples loaded]' % idx)
        sys.stdout.flush()
        yield item
    sys.stdout.write('\n')


def test_results(y, y_res, lbl):
    cm = confusion_matrix(y, y_res, labels=lbl)
    cm = cm / cm.astype(np.float).sum(axis=1)
    print classification_report(y, y_res, labels=lbl)

    # Show confusion matrix in a separate window
    pl.matshow(cm)
    pl.title('Confusion matrix')
    pl.colorbar()
    pl.ylabel('True label')
    pl.xlabel('Predicted label')
    pl.show()


def test():

    query_images = get_query_images()
    for key, image in query_images.iteritems():
        query_images[key][ 0: 18,  0:100] = [255, 255, 255]
        query_images[key][82:100,  0:100] = [255, 255, 255]
        query_images[key][ 0:100,  0: 18] = [255, 255, 255]
        query_images[key][ 0:100, 82:100] = [255, 255, 255]


    # Loop over test data
    X = []
    y = []
    y_res = []
    labels = []
    for button in progressor(buttons.find(snapshot=True)):
        try:
            image = preprocess(convert(button['image'], input_type='ros', output_type='cv2'))
            max_score = 0
            label = cls = 'NONE'
            x = np.zeros(len(query_images))
            for idx, (key, qimage) in enumerate(query_images.iteritems()):
                score = keypoint_match(qimage, image)
                x[idx] = score
                if score > max_score:
                    max_score = score
                    cls = key
            if max_score > 0.025:
                label = cls
            X.append(x)
            y.append(button['label'].upper() if button['label'] is not None else 'NONE')
            y_res.append(label.upper())
            if y[-1] not in labels:
                labels.append(y[-1])
        except:
            print 'Skipping sample'
        #print x
        #print label + ' ' + str(max_score)
        #cv2.imshow('t', image)
        #cv2.waitKey()
    test_results(y, y_res, labels)
    clf = LogisticRegression()
    print labels
    train_save_test(clf, X, y, 'temp', labels)


def main():
    test()

if __name__ == "__main__":
    main()
