import time
from turtle import width
import numpy as np
import math as m
import random
from PIL import Image

t = time.perf_counter()

height, width = (1080, 1960)  #guidescope dimensions
sky_dimensions = (8640, 15680)  #full image dimensions
max_radius = 10                 #max radius of any star
margin = 20                     #margin on edge of image
decay_constant = 8              #decay constant for the brightness function
freq = 10000                      #pixels per star
img_max = 255                   #max saturation value

#create a numpy array of zeros
#will store luminosity values 0 - 255
sky = np.zeros(sky_dimensions)
print(sky.shape)

#value to define the brightness of the star 
#luminosity varies across the star radius according to an exponential
#total luminosity also varies based on magnitude (some factor of value 0 - 1)
#calculate_luminosity = lambda x, r, mag: max((((r**2-x)/r**2)**exponent)*mag, 0)
calculate_luminosity = lambda x, r, mag: max(m.exp(-decay_constant*(x/r**2))*mag, 0)

for i in range(margin, sky.shape[0]-margin):
    for j in range(margin, sky.shape[1]-margin):
        if random.randint(1, freq) == freq:
            star = True
        else:
            star = False
        if star:
            radius = random.randint(1, max_radius)
            magnitude = random.uniform(0.2, 1)
            for k in range(i-radius, i+radius+1):
                for l in range(j-radius, j+radius+1):
                    distance = min((k-i)**2 + (l-j)**2, radius**2)
                    luminosity = calculate_luminosity(distance, radius, magnitude) * img_max
                    sky[k, l] = min(luminosity+sky[k, l], 255)
    if i % 500 == 0:
        print(f"{(i*100)/sky.shape[1]}%")

im = Image.fromarray(sky)
im = im.convert("L")
im.save("sky.png")

im2 = Image.fromarray(sky[:height, :width])
im2 = im2.convert("L")
im2.save("frame.png")

print(time.perf_counter() - t)
print("Done.")

