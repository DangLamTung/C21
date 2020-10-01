#!/usr/bin/python3
# 2018.01.04 13:01:24 CST
# 2018.01.04 14:42:58 CST

import cv2
import numpy as np
import os
img = cv2.imread("test.jpg")
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
ret,threshed = cv2.threshold(gray,100,255,cv2.THRESH_BINARY)

# find contours without approx
cnts = cv2.findContours(threshed,cv2.RETR_LIST,cv2.CHAIN_APPROX_NONE)[-2]

# get the max-area contour
cnt = sorted(cnts, key=cv2.contourArea)[-1]

# calc arclentgh
arclen = cv2.arcLength(cnt, True)

# do approx
eps = 0.00005
epsilon = arclen * eps
approx = cv2.approxPolyDP(cnt, epsilon, True)

# draw the result
canvas = img.copy()
for pt in approx:
    cv2.circle(canvas, (pt[0][0], pt[0][1]), 7, (0,255,0), -1)

cv2.drawContours(canvas, [approx], -1, (0,0,255), 2, cv2.LINE_AA)

# save
cv2.imwrite("result.png", canvas)