__author__ = 'rolf'

from pymongo import MongoClient
import cv2
import numpy as np
import sys

db = MongoClient()
buttons = db['aidu']['elevator_buttons']
untested = db['aidu']['elevator_buttons_untested']

def get_label(key):
    if key == ord('b'):
        return 'B'
    elif key == ord('k'):
        return 'K'
    elif key == ord('1'):
        return '1'
    elif key == ord('2'):
        return '2'
    elif key == ord('3'):
        return '3'
    elif key == ord('4'):
        return '4'
    elif key == ord('u'):
        return 'up'
    elif key == ord('d'):
        return 'down'
    elif key == ord('n'):
        return 'none'
    else:
        return None

try:
    start = sys.argv[1]
except:
    start = 0

for idx, button in enumerate(untested.find({}).sort("_id", 1)):
    if idx < int(start):
        continue
    elif idx == int(start) and int(start) is not 0:
        print 'starting from %d' % idx
    x = np.array( button['img'], dtype=np.uint8 )
    img = cv2.imdecode(x, 1)
    cv2.putText(img, str('%s' % button['label']), (2,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 255)
    cv2.imshow('test', img)
    key = cv2.waitKey() & 255
    label = get_label(key)
    if key == 27:
        print 'bye!'
        break
    elif label is not None:
        if label is 'none':
            label = None
        print '%d - assigned label %s to image %s' % (idx, label, button.get('file'))
        #button['label'] = label
        #buttons.save(button)
        buttons.save({'img': button['img'], 'label': label})
    else:
        print '%d - assigned no label change to image %s' % (idx, button.get('file'))
