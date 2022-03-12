from collections import deque
from datetime import datetime
from PIL import Image, ImageTk
import numpy as np
import tkinter as tk
import threading
import serial
import time
import math

import os
os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"

import cv2
from scipy.optimize import linear_sum_assignment

#class for object tracking
class Tracker:
    def __init__(self):
        #store info on objects in a dictionary
        #id : x, y, width, height, dx, dy, memory
        self.previous_objects = {}
        self.objects = {}

        #info for the tracked object
        self.object_id = -1
        self.object_pos = (0, 0)

    def update(self, detection):
        #number of frames missing objects are remembered for
        MEMORY = 100
        #an artificial cost to add to the list 
        #matches with missing objects
        THRESHOLD = 20

        #clear current list
        self.objects = {}

        #defines a cost function for the graph
        #uses euclidean distance
        #has dummy nodes to match to incase of new objects
        cost =  [[((t_x - d_x)**2 + (t_y - d_y)**2)**0.5 
                    for t_x, t_y, _, _, _, _, _ in self.previous_objects.values()] 
                    for d_x, d_y, _, _ in detection]
        cost = [(*costs, *([THRESHOLD]*len(detection))) for costs in cost]

        #special case of empty list (for when there are no current detections)
        if not cost:
            cost = [[]]

        #uses the hungarian algorithm to create a minimum cost sum of the matches
        matches = dict(zip(*linear_sum_assignment(cost)))

        #new id iterator
        new_id = max(self.previous_objects.keys(), default=-1)

        for key, value in matches.items():
            #for new objects
            if value >= len(self.previous_objects):
                #set the value to a new id
                new_id += 1
                matches[key] = new_id
            #for matched objects
            elif value < len(self.previous_objects):
                #set the value to the matched id
                matches[key] = list(self.previous_objects.keys())[value]

        for key, value in matches.items():
            #for matched objects
            if value in self.previous_objects.keys():
                #compute the change in position
                #and set memory to 0
                self.objects[value] = (*detection[key], 
                                        detection[key][0] - self.previous_objects[value][0],
                                        detection[key][1] - self.previous_objects[value][1],
                                        0)
            #for new objects
            elif value not in self.previous_objects.keys():
                #set dx, dy = 0, 0
                #and set memory to 0
                self.objects[value] = (*detection[key], 0, 0, 0)

        for key, value in self.previous_objects.items():
            #for old objects that arent matched in the new frame
            #and havent been lost for 10 or more frames
            #compute their new position using x += dx, y += dy
            #rectangle h, w = 0, 0
            if key not in self.objects.keys() and value[6] < MEMORY:
                self.objects[key] = (value[0] + value[4], value[1] + value[5],
                                        0, 0, value[4], value[5], value[6] + 1)

        self.previous_objects = self.objects

    def clear(self):
        self.objects = {}
        self.previous_objects = {}

class Gui(tk.Frame):
    def __init__(self):
        self.root = tk.Tk()
        self.panel = None
        self.open = True

        self.root.wm_title("Tracking Control Panel")
        self.root.wm_protocol("WM_DELETE_WINDOW", self.on_close)

    def update_video(self, frame):
        # OpenCV represents images in BGR order; however PIL
        # represents images in RGB order, so we need to swap
        # the channels, then convert to PIL and ImageTk format
        image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image = Image.fromarray(image)
        image = ImageTk.PhotoImage(image)

        # if the panel is not None, we need to initialize it
        if self.panel is None:
            print("Initialise GUI")
            self.panel = tk.Label(image=image)
            self.panel.image = image
            self.panel.pack(side="left", padx=10, pady=10)

        # otherwise, simply update the panel
        else:
            self.panel.configure(image=image)
            self.panel.image = image
    
    def on_close(self):
        print("Exiting...")
        self.open = not self.open
        self.root.destroy()
        self.root.quit()

def send_motor(motor, steps):
    # commands are enclosed with <>
    # a/b for azimuth/elevation motor respectively
    # +/- for clockwise/anticlockwise respectively
    # positive integer for number of steps
    # e.g. <a+1000>; azimuth, clockwise, 1000 steps

    cmd = "<"

    if motor == "az":
        cmd += "a"
    elif motor == "elv":
        cmd += "b"
    else:
        print("motor error")
    
    if steps < 0:
        cmd += "-"
    elif steps >= 0:
        cmd += "+"
    else:   
        print("directional error")
    
    cmd += str(abs(steps))
    cmd += ">"
    print(f"sending: {cmd}")

    #commands must be encoded using utf-8
    #command is then sent to the arduino
    #then close the serial communications

    cmd = cmd.encode("utf-8")
    ser.write(cmd)
    time.sleep(1)

def get_bounding_rectangles(frame):
    #greyscale and blur both the current and previous frame
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    frame = cv2.GaussianBlur(frame, (11,11), 0)

    #take threshold of frame
    #dilate and erode to join fragmented objects
    #get a list of edges and their positions
    _, threshold = cv2.threshold(frame, 20, 255, cv2.THRESH_BINARY)
    dilated = cv2.dilate(threshold, None, iterations = 1)
    eroded = cv2.erode(dilated, None, iterations = 1)
    edges, _ = cv2.findContours(eroded, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    #size restrictions for what constitutes an object
    min_area = 10
    max_area = 2000000

    #initialise an empty list to hold object bounding rectangles, (x, y, width, height)
    object_bounding_rectangles = []

    #loop through list of valid objects and store their positions
    for edge in edges:
        if min_area < cv2.contourArea(edge)  < max_area:
            object_bounding_rectangles.append(cv2.boundingRect(edge))

    return object_bounding_rectangles

def main():
    #select the communication port and open
    #ser = serial.Serial("COM3", 9600)
    #ser.timeout = 1
    #if not ser.isOpen():
    #    ser.open()

    # main setup
    print("Starting main...")

    # GUI setup
    gui = Gui()

    # define camera dimensions
    guide_dimensions = (1920, 1080)
    main_dimensions = (3856, 2764)

    # define video output settings
    codec = cv2.VideoWriter_fourcc(*'WMV3')
    fps = 20
    guide_out = cv2.VideoWriter('guide_out.wmv', codec, fps, guide_dimensions)
    #main_out = cv2.VideoWriter('main_out.wmv', codec, fps, guide_dimensions)

    # define camera inputs
    #guide_in = cv2.VideoCapture(0, cv2.CAP_ANY)
    guide_in = cv2.VideoCapture("guidescope_test.mp4")
    guide_in.set(cv2.CAP_PROP_FRAME_WIDTH, guide_dimensions[0])
    guide_in.set(cv2.CAP_PROP_FRAME_HEIGHT, guide_dimensions[1])
    guide_in.set(cv2.CAP_PROP_FOURCC, codec)
    guide_dimensions = (int(guide_in.get(cv2.CAP_PROP_FRAME_WIDTH)),
                        int(guide_in.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    print("Guidescope open")

    #main_in = cv2.VideoCapture(1, cv2.CAP_ANY)
    #main_in = cv2.VideoCapture("test_in4.mp4")
    #main_in.set(cv2.CAP_PROP_FRAME_WIDTH, main_dimensions[0])
    #main_in.set(cv2.CAP_PROP_FRAME_HEIGHT, main_dimensions[1])
    #main_in.set(cv2.CAP_PROP_CONVERT_RGB, 0.0)
    #main_in.set(cv2.CAP_PROP_FOURCC, codec)
    #print("Main Camera open")

    #CALIBRATE

    #GO TO ISS

    # initialise euclidean distance tracker object
    tracker = Tracker()
    # initialise object tracking as false
    tracking = False
    # initialise camera recording as false
    recording = False
    # initialise object tracking overlay as true
    overlay = True

    # main tracking loop
    # run until user closes GUI
    while gui.open:
        # start timer for fps
        loop_time = time.perf_counter()

        # get new frame
        ret, guide_frame = guide_in.read()
        guide_frame = cv2.rotate(guide_frame, cv2.ROTATE_180)

        # break if there is no new frame;
        # i.e. if the test video ends
        if not ret:
            break

        #_ , main_frame = main_in.read()

        # get a list of bounding rectangles for each object in frame
        object_bounding_rectangles = get_bounding_rectangles(guide_frame)

        # get a list of objects with ids
        tracker.update(object_bounding_rectangles)

        # move the motors if we are tracking an object
        if tracking:
            x, y = tracker.object_pos
            x_mov = guide_dimensions[0]/2 - x
            y_mov = guide_dimensions[1]/2 - y
            #print(f"{x_mov}, {y_mov}")


        # user keyboard interactions
        key = cv2.waitKey(1)

        # exit with e
        if key == ord('e'):
            print("Exiting...")
            break

        # save screenshot with [space]
        if key == ord(' '):
            file_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S-%f")
            #cv2.imwrite(f".\Screenshots\main_{file_time}.png", main_frame)
            cv2.imwrite(f".\Screenshots\guide_{file_time}.png", guide_frame)
            print(f"Saving screenchot as \"{file_time}\"")
        
        # toggle tracking with t
        if key == ord('t'):
            tracking = not tracking

        # toggle recording with r
        if key == ord('r'):
            recording = not recording

        # toggle overlay with o
        if key == ord('o'):
            overlay = not overlay

        # clear ids with c
        if key == ord('c'):
            tracker.clear()

        # increment tracking id with w, decrement with s
        # increment by 10 with q, decrement by 10 with a
        if key == ord('w'):
            tracker.object_id += 1
        elif key == ord('s'):
            tracker.object_id -= 1
            if tracker.object_id < 0:
                tracker.object_id = 0

        if key == ord('q'):
            tracker.object_id += 10
        elif key == ord('a'):
            tracker.object_id -= 10
            if tracker.object_id < 0:
                tracker.object_id = 0

        if overlay:
            for id, obj in tracker.objects.items():
                x, y, width, height, dx, dy, mem = obj

                # if this is the object we are tracking
                # then change the colour from blue to red
                # else leave it as blue
                if id == tracker.object_id and tracking:
                    tracker.object_pos = (x, y)
                    text_colour = (0, 0, 255)
                    box_colour = (0, 0, 255)
                else:
                    text_colour = (255, 0, 0)
                    box_colour = (0, 255, 0)

                if not width*height == 0:
                    cv2.putText(guide_frame, str(id), (x, y - 5), cv2.FONT_HERSHEY_PLAIN, 1, text_colour, 2)
                    cv2.rectangle(guide_frame, (x, y), (x + width, y + height), box_colour, 1)

            # centre circle in the guidescope
            (x, y) = guide_dimensions
            cv2.circle(guide_frame, (x//2, y//2), 5, (0, 0, 255), -1)

            # outline main camera FOV in the guidescope
            # main FOV 0.3056deg x 0.2196deg
            # guide FOV 1.91deg x 1.07deg
            # thus the main FOV is bounded by the centre 20.5% x 16% of the guide frame
            (h, w) = (int(0.205*y), int(0.16*x))
            cv2.rectangle(guide_frame, ((x-w)//2, (y-h)//2), ((x+w)//2, (y+h)//2), (0, 0, 255), 1)

            # write tracking id on frame
            if tracking:
                guide_frame = cv2.putText(guide_frame, f"Tracking", 
                                            (5, guide_dimensions[1]-25), 
                                            cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)
            guide_frame = cv2.putText(guide_frame, f"Tracking id: {tracker.object_id}", 
                                        (5, guide_dimensions[1]-10), 
                                        cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)

            # update fps counter
            loop_time = time.perf_counter() - loop_time
            cv2.putText(guide_frame, f"{1/loop_time:02.2f} FPS", (5, 15), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)

            if recording:
                # write 'recording' in bottom left of frame to notify user the video feed is being recorded
                cv2.putText(guide_frame, f"Recording", (5, 30), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)

        if recording:
            # save frame to video file
            guide_out.write(guide_frame)
            #main_out.write(cv2.resize(main_frame, guide_dimensions))

        # downsize the resolution before showing to the user
        gui_frame = cv2.resize(guide_frame, (1024, 576))
        gui.update_video(gui_frame)
        gui.root.update()

    guide_out.release()
    #main_out.release()
    #ser.close()

    print("Done.")