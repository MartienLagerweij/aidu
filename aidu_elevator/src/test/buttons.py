# matching features of two images
import cv2
import cv
import sys
import scipy as sp
import numpy as np

img1_path = '/home/rolf/Dropbox/Robotica/World Perception/Assignment 6/Liftknop Fotos/up_template_empty.png'
img2_path =  '/home/rolf/Dropbox/Robotica/World Perception/Assignment 6/Liftknop Fotos/lift-front-6.jpg' 

img1 = cv2.imread(img1_path, cv2.CV_LOAD_IMAGE_GRAYSCALE)
img2 = cv2.imread(img2_path, cv2.CV_LOAD_IMAGE_GRAYSCALE)
ratio1 = img2.shape[0] / 640.0
ratio2 = img2.shape[1] / 480.0
ratio = ratio1 if ratio1 < ratio2 else ratio2
img2 = cv2.resize(img2, (0,0), fx=ratio, fy=ratio)

# Laplacian Gradients
img1 = cv2.Laplacian(img1,cv2.CV_8UC4)
img2 = cv2.Laplacian(img2,cv2.CV_8UC4)

# Canny Edge Detection
#img1 = cv2.Canny(img1,100,200)
#img2 = cv2.Canny(img2, 100, 200)

# Hough lines
minLineLength = 50
maxLineGap = 10
#img1 = cv2.HoughLinesP(img1,1,np.pi/180,100,minLineLength,maxLineGap)
#img2 = cv2.HoughLinesP(img2,1,np.pi/180,100,minLineLength,maxLineGap)

# Convert to grayscale
#img1 = cv2.cvtColor(img1,cv2.COLOR_BGR2GRAY)
#img2 = cv2.cvtColor(img2,cv2.COLOR_BGR2GRAY)

# Initiate SIFT detector
sift = cv2.SIFT()

# find the keypoints and descriptors with SIFT
k1, des1 = sift.detectAndCompute(img1,None)
k2, des2 = sift.detectAndCompute(img2,None)

# BFMatcher with default params
bf = cv2.BFMatcher()
matches = bf.knnMatch(des1,des2, k=2)

sel_matches = []
for m,n in matches:
    if m.distance < 0.90*n.distance:
        sel_matches.append(m)


# #####################################
# visualization
h1, w1 = img1.shape[:2]
h2, w2 = img2.shape[:2]
view = sp.zeros((max(h1, h2), w1 + w2, 3), sp.uint8)
view[:h1, :w1, 0] = img1
view[:h2, w1:, 0] = img2
view[:, :, 1] = view[:, :, 0]
view[:, :, 2] = view[:, :, 0]

for m in sel_matches:
    # draw the keypoints
    # print m.queryIdx, m.trainIdx, m.distance
    color = tuple([int(sp.random.randint(0, 255)) for _ in xrange(3)])
    cv2.line(view, (int(k1[m.queryIdx].pt[0]), int(k1[m.queryIdx].pt[1])), (int(k2[m.trainIdx].pt[0] + w1), int(k2[m.trainIdx].pt[1])), color)

cv2.imshow("view", view)
cv2.waitKey()


