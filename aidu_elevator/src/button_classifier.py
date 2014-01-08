#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import os
import pylab as pl
import sys
from sensor_msgs.msg import CompressedImage
from aidu_elevator.msg import Button
from pymongo import MongoClient
from sklearn.linear_model import LogisticRegression
from sklearn.multiclass import OneVsRestClassifier
from sklearn.metrics import classification_report, confusion_matrix
from sklearn.cross_validation import train_test_split
from sklearn.externals import joblib
from sklearn.svm import SVC
from sklearn.preprocessing import MinMaxScaler
from sklearn.pipeline import Pipeline
from time import clock

db = MongoClient()
db_buttons = db['aidu']['elevator_buttons']
db_untested = db['aidu']['elevator_buttons_untested']

labels = []
data_path = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../data/'))
model_directory = os.path.abspath(os.path.join(data_path, 'classification_model'))
label_map = {'1': Button.BUTTON_1,
             '2': Button.BUTTON_2,
             '3': Button.BUTTON_3,
             '4': Button.BUTTON_4,
             'B': Button.BUTTON_B,
             'K': Button.BUTTON_K,
             'up': Button.BUTTON_UP,
             'down': Button.BUTTON_DOWN,
             'none': Button.BUTTON_NONE}
inverse_label_map = {v: k for k, v in label_map.items()}

def progressor(generator):
    for idx, item in enumerate(generator):
        sys.stdout.write('\r[%d samples loaded]' % idx)
        sys.stdout.flush()
        yield item
    sys.stdout.write('\n')


def get_mongo_image(image):
    img = cv2.imencode('.jpg', image)[1]
    return np.array(img).tolist()


def get_image(image_message):
    np_arr = np.fromstring(image_message.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
    return image_np


def process_image(image):
    image = image[20:80,20:80]
    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    #image = cv2.medianBlur(image, 5)

    # Canny edge detection
    #output = cv2.Canny(image,100,200)

    # Adapative gaussian threshold
    output = cv2.adaptiveThreshold(image, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
    return 255 - output


def process_label(label):
    if label is None:
        label = 'none'
    if label not in labels:
        labels.append(label)
    #return labels.index(label)
    return label


def get_feature_vector(image):
    image = process_image(image)
    image = cv2.medianBlur(image, 5)

    #size = (60, 60, 0, 0)
    #for y, row in enumerate(image):
    #    for x, px in enumerate(row):
    #        if px > 0:
    #            size = (min(size[0], max(x-1,0)), min(size[1], max(y-1,0)),
    #                    max(size[2], min(x+1,60)), max(size[3], min(y+1,60)))
    #if size == (60, 60, 0, 0):
    #    size = (0, 0, 60, 60)
    #image = image[size[1]:size[3], size[0]:size[2]]
    #image = cv2.resize(image, (50, 50))
    #image = cv2.adaptiveThreshold(image, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)

    #cv2.imshow('t', image)
    #cv2.waitKey()

    return image.flatten().astype(np.float)


def train():
    clf = Pipeline([
        ('normalizer', MinMaxScaler()),
        ('classifier', OneVsRestClassifier(LogisticRegression(penalty='l2', C=1e-1)))] # LogisticRegression(penalty='l2', C=1e-4)
    )
    X = []
    y = []
    print 'Getting data'
    for button in progressor(db_buttons.find({})):
        x = np.array( button['img'], dtype=np.uint8 )
        img = cv2.imdecode(x, 1)
        vector = get_feature_vector(img)
        #print vector
        X.append(vector)
        y.append(process_label(button['label']))

    print 'Training'
    clf.fit(X,y)

    print 'Saving to file'
    joblib.dump(clf, os.path.join(model_directory, 'ovr_lr.pkl'), compress=9)

    print 'Testing'
    X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.4)
    clf.fit(X_train,y_train)
    pred = clf.predict(X_test)
    cm = confusion_matrix(y_test, pred, labels=labels)
    cm = cm / cm.astype(np.float).sum(axis=1)
    print classification_report(y_test, pred, labels=labels)

    # Show confusion matrix in a separate window
    pl.matshow(cm)
    pl.title('Confusion matrix')
    pl.colorbar()
    pl.ylabel('True label')
    pl.xlabel('Predicted label')
    pl.show()

def assign_message_label(button_message, label):
    button_message.button_type = label_map.get(label)


def display_button(button, label_str):
    img = get_image(button.image)
    cv2.putText(img,label_str, (2,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 255)
    cv2.imshow('Button', img)
    cv2.waitKey(10)


def callback(button):
    try:
        img = get_image(button.image)
        vector = get_feature_vector(img)
        label = clf.predict([vector])[0]
        try:
            p = np.max(clf.predict_proba([vector])[0])
        except:
            p = 1.0
        rospy.loginfo('%s - %.3f' % (label, p))
        assign_message_label(button, label)
        if button.button_type != button.BUTTON_NONE and p > 0.7:
            label_str = '%s (%.0f%%)' % (label, 100.0*p)
            button.on = False
            display_button(button, label_str)
            button_publisher.publish(button)
            #db_untested.insert({'img': get_mongo_image(img), 'label': label})
    except Exception as e:
        rospy.logwarn(e)


def run_node():
    global clf, button_publisher
    try:
        cv2.namedWindow('Button', cv2.WINDOW_NORMAL)
        clf = joblib.load(os.path.join(model_directory, 'ovr_lr.pkl'))
        rospy.init_node('button_classifier', anonymous=True)
        rospy.Subscriber("/elevator/button", Button, callback, queue_size=6)
        button_publisher = rospy.Publisher("/elevator/button/classified", Button)
        rospy.spin()
    except Exception as e:
        rospy.logerr(e)
        return


def main():
    if len(sys.argv) == 2 and sys.argv[1] == 'train':
        train()
    else:
        run_node()

if __name__ == "__main__":
    main()