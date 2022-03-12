import cv2
import time
from matplotlib import image
import numpy as np
import math as m
import random
from PIL import Image

sky = cv2.imread("sky.png", cv2.IMREAD_GRAYSCALE)
iss = cv2.imread("ISS_small.png", cv2.IMREAD_GRAYSCALE)

margin = 60
h, w = (1080, 1920)    #guidescope dimensions

codec = cv2.VideoWriter_fourcc(*'MP4V')
fps = 20
out = cv2.VideoWriter('new_test.mp4', codec, fps, (w, h), False)

iss_path = lambda x: -((x-margin)/(160))**2+(sky.shape[0]-h)

for j in range(60, sky.shape[1] - w):
    i = int(iss_path(j))
    frame = np.copy(sky[i:i+h, j:j+w])
    iss_pos = ((h-iss.shape[0])//2, (w-iss.shape[1])//2)
    for k in range(0, iss.shape[0]):
        for l in range(0, iss.shape[1]):
            frame[iss_pos[0]+k, iss_pos[1]+l] += iss[k, l]
            frame[iss_pos[0]+k, iss_pos[1]+l] = min(frame[iss_pos[0]+k, iss_pos[1]+l], 255)
    #cv2.imshow("Guide Scope", frame)
    #cv2.waitKey(1)
    #print(f"{frame.shape}, {(h, w)}")
    out.write(frame)

cv2.destroyAllWindows()
out.release()