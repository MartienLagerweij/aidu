#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import os
import pylab as pl
import sys
from sensor_msgs.msg import CompressedImage
from aidu_elevator.msg import Button
from images import convert
from pymongo import MongoClient
from sklearn.linear_model import LogisticRegression, SGDClassifier
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
certainties = {k: 0 for v, k in label_map.iteritems()}
inverse_label_map = {v: k for k, v in label_map.items()}


def threshold(image):

    # Pre process image
    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Apply adapative gaussian threshold
    image = cv2.adaptiveThreshold(image, 225, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 21, 3)
    image = cv2.medianBlur(image, 3)

    # Return output
    return 255 - image


def progressor(generator):
    for idx, item in enumerate(generator):
        sys.stdout.write('\r[%d samples loaded]' % idx)
        sys.stdout.flush()
        yield item
    sys.stdout.write('\n')


def process_label(label):
    if label is None:
        label = 'none'
    if label not in labels:
        labels.append(label)
    #return labels.index(label)
    return label


def get_feature_vector(image):
    image = threshold(image)
    image = cv2.medianBlur(image, 5)
    return image.flatten().astype(np.float)


def get_onoff_feature_vector(image):
    image = np.array(image)
    image[20:80, 20:80] = [0, 0, 0]
    ORANGE_MIN = np.array([12, 80, 100],np.uint8)
    ORANGE_MAX = np.array([26, 255, 215],np.uint8)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    image = cv2.inRange(image, ORANGE_MIN, ORANGE_MAX)
    return image.flatten().astype(np.float)


def train_save_test(clf, X, y, filename, lbl):
    print 'Training'
    clf.fit(X,y)

    print 'Saving to file'
    joblib.dump(clf, os.path.join(model_directory, filename), compress=9)

    print 'Testing'
    X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.4)
    clf.fit(X_train,y_train)
    pred = clf.predict(X_test)
    cm = confusion_matrix(y_test, pred, labels=lbl)
    cm = cm / cm.astype(np.float).sum(axis=1)
    print classification_report(y_test, pred, labels=lbl)

    # Show confusion matrix in a separate window
    pl.matshow(cm)
    pl.title('Confusion matrix')
    pl.colorbar()
    pl.ylabel('True label')
    pl.xlabel('Predicted label')
    pl.show()


def train():

    print 'Button identifier classifier'
    clf = Pipeline([
        ('normalizer', MinMaxScaler()),
        ('classifier', OneVsRestClassifier(LogisticRegression(penalty='l2', C=1e-1)))] # LogisticRegression(penalty='l2', C=1e-4)
    )
    X = []
    y = []
    print 'Getting data'
    n = 0
    for button in progressor(db_buttons.find({})):
        #x = np.array( button['image'], dtype=np.uint8 )
        #img = cv2.imdecode(x, 1)
        img = convert(button['image'], input_type='ros', output_type='cv2')
        vector = get_feature_vector(img)
        display_button(img, button['label'], '')
        key = cv2.waitKey() % 256
        if(key == ord('s')):
            n += 1
            cv2.imwrite('/home/rolf/Desktop/%d.jpg' % n, img)
        #print vector
        X.append(vector)
        y.append(process_label(button['label']))

    train_save_test(clf, X, y, 'ovr_lr.pkl', labels)

    print 'Button on/off classifier'

    X = []
    y = []
    print 'Getting data'
    for button in progressor(db_buttons.find()):
        x = np.array( button['img'], dtype=np.uint8 )
        img = cv2.imdecode(x, 1)

        #print button.get('on')
        vector = get_onoff_feature_vector(img)
        #print sum(vector)
        #print len(vector)
        X.append(vector)
        y.append(1 if button.get('on') == True else 0)

    clf = Pipeline([
        ('normalizer', MinMaxScaler()),
        ('classifier', LogisticRegression(penalty='l2', C=1e1, class_weight='auto'))]#LogisticRegression(penalty='l2', C=1e0))]
    )

    train_save_test(clf, X, y, 'onoff_lr.pkl', [0,1])


def assign_message_label(button_message, label):
    button_message.button_type = label_map.get(label)


def display_button(img, label_str, onoff_str=''):
    #img = convert(img, input_type='ros', output_type='cv2')
    cv2.putText(img, label_str, (2,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 255)
    cv2.putText(img, onoff_str, (2,94), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 255)
    cv2.imshow('Button', img)
    cv2.waitKey(10)


def callback(button):
    try:
        img = convert(button.image.data, input_type='ros', output_type='cv2')
        vector = get_feature_vector(img)
        label = clf.predict([vector])[0]

        try:
            p = np.max(clf.predict_proba([vector])[0])
        except:
            p = 1.0
	
        #rospy.loginfo('%s: %f' % (label, p))

        assign_message_label(button, label)
        #display_button(img, label, '%f' % p)

        for k, v in certainties.iteritems():
            certainties[k] = max(0, certainties[k] - 1)
	
        if button.button_type != button.BUTTON_NONE and p > 0.7:
            certainties[button.button_type] = min(10, certainties[k] + 3 * p)
            if certainties[button.button_type] > 6:
                #rospy.loginfo('%s - %.3f' % (label, p))
                on = onoff_clf.predict(get_onoff_feature_vector(img))
                button.on = True if on else False
                label_str = '%s (%.0f%%)' % (label, 100.0*p)
                onoff_str = 'on' if on else 'off'
                button_publisher.publish(button)

    except Exception as e:
        rospy.logwarn(e)


def run_node():
    global clf, onoff_clf, button_publisher
    try:
        clf = joblib.load(os.path.join(model_directory, 'ovr_lr.pkl'))
        onoff_clf = joblib.load(os.path.join(model_directory, 'onoff_lr.pkl'))
        rospy.init_node('button_classifier', anonymous=True)
        rospy.Subscriber("/elevator/button", Button, callback)
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