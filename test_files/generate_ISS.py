import cv2
import time
import numpy as np
import math as m
import random
from PIL import Image

frame = cv2.imread("ISS.jpg")
frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
frame = cv2.GaussianBlur(frame, (11,11), 0)

_, threshold = cv2.threshold(frame, 50, 255, cv2.THRESH_BINARY)
dilated = cv2.dilate(threshold, None, iterations = 1)
eroded = cv2.erode(dilated, None, iterations = 1)
edges, _ = cv2.findContours(eroded, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
x, y, w, h = cv2.boundingRect(edges[0])
margin = 5
w += margin
h += margin

frame[frame < 50] = 0

aoi_big = frame[y:y+h, x:x+w]
aoi_small = cv2.resize(aoi_big, (24, 12))

im = Image.fromarray(aoi_big)
im = im.convert("L")
im.save("ISS_big.png")

im = Image.fromarray(aoi_small)
im = im.convert("L")
im.save("ISS_small.png")