from collections import deque
from datetime import datetime
from PIL import Image, ImageTk
import numpy as np
import tkinter as tk
from pandas_datareader import test
import serial
import time

import os
os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"

import cv2
from scipy.optimize import linear_sum_assignment

# global states
# camera specs
GUIDESCOPE_FOV_ARCSEC = (6876, 3852)
GUIDESCOPE_RESOLUTION = (1920, 1080)

MAINCAM_FOV_ARCSEC = (1100.16, 790.56)
MAINCAM_RESOLUTION = (3856, 2764)

#select the communication port and open
ser = serial.Serial("COM3", 9600)
ser.timeout = 1
if not ser.isOpen():
    ser.open()
    time.sleep(2)

# tracker object
# tracks objects across frames
class Tracker:
    def __init__(self):
        #store info on objects in a dictionary
        #id : x, y, width, height, dx, dy, memory
        self.previous_objects = {}
        self.objects = {}

        # info for the target object
        self.tracking = False
        self.target_id = 57
        self.target_pos = (0, 0)

    # toggle the tracking param
    # when True, the motors will follow the target object
    def track(self):
        self.tracking = not self.tracking

    # assign ids to the objects in a new frame
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

    # clear the memory of the tracker
    def clear(self):
        self.objects = {}
        self.previous_objects = {}

# gui object
# handles GUI
class Gui():

    def overlay_command(self):
        self.overlay = not self.overlay
    
    def record_command(self):
        self.recording = not self.recording
    
    def screenshot_command(self):
        file_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S-%f")
        cv2.imwrite(f".\Screenshots\main_{file_time}.png", self.main_frame)
        cv2.imwrite(f".\screenshots\guide_{file_time}.png", self.guide_frame)
        print(f"Saving screenchot as \"{file_time}\"")
    
    def motor_command(self, motor, dir):
        steps = abs(int(self.steps_entry.get()))
        if not dir:
            steps *= -1
        try:
            move_motor(motor, steps)
        except ValueError as e:
            print(e)
    
    def source_command(self):
        self.source_id = not self.source_id

    def __init__(self, tracker):
        self.tracker = tracker

        self.root = tk.Tk()
        self.open = True

        self.pady = 1
        self.padx = 1

        self.guide_frame = None
        self.main_frame = None
        self.show_frame = None

        self.video_frame = None
        self.video_width = 4

        # video options
        self.video_label = tk.Label(self.root, text="Video Options", font='Helvetica 18 bold')
        self.video_label.grid(row=0, column=self.video_width+1, padx=self.padx, pady=self.pady)

        self.overlay = True
        self.overlay_button = tk.Button(self.root, text="Toggle Overlay", command=self.overlay_command)
        self.overlay_button.grid(row=1, column=self.video_width+1, padx=self.padx, pady=self.pady)

        self.recording = False
        self.record_button = tk.Button(self.root, text="Toggle Recording", command=self.record_command)
        self.record_button.grid(row=2, column=self.video_width+1, padx=self.padx, pady=self.pady)

        self.screenshot = False
        self.screenshot_button = tk.Button(self.root, text="Screenshot", command=self.screenshot_command)
        self.screenshot_button.grid(row=2, column=self.video_width+2, padx=self.padx, pady=self.pady)

        self.source_id = False
        self.source_button = tk.Button(self.root, text="Toggle Source", command=self.source_command)
        self.source_button.grid(row=1, column=self.video_width+2, padx=self.padx, pady=self.pady)

        # tracking options
        self.tracking_label = tk.Label(self.root, text="Tracking Options", font='Helvetica 18 bold')
        self.tracking_label.grid(row=4, column=self.video_width+1, padx=self.padx, pady=self.pady)

        self.clear_button = tk.Button(self.root, text="Reset Object IDs", command=self.tracker.clear)
        self.clear_button.grid(row=5, column=self.video_width+2, padx=self.padx, pady=self.pady)

        self.tracking = False
        self.tracking_button = tk.Button(self.root, text="Toggle Tracking Mode", command=self.tracker.track)
        self.tracking_button.grid(row=5, column=self.video_width+1, padx=self.padx, pady=self.pady)

        self.tracking_id_label = tk.Label(self.root, text="Tracking ID")
        self.tracking_id_label.grid(row=6, column=self.video_width+1, padx=self.padx, pady=self.pady)

        self.tracking_id_input = tk.Entry(self.root, justify="right", width=18)
        self.tracking_id_input.insert(tk.END, "0")
        self.tracking_id_input.grid(row=6, column=self.video_width+2, padx=self.padx, pady=self.pady)

        self.min_threshold_label = tk.Label(self.root, text="Minimum Threshold Value")
        self.min_threshold_label.grid(row=7, column=self.video_width+1, padx=self.padx, pady=self.pady)

        self.min_threshold_input = tk.Entry(self.root, justify="right", width=18)
        self.min_threshold_input.insert(tk.END, "20")
        self.min_threshold_input.grid(row=7, column=self.video_width+2, padx=self.padx, pady=self.pady)

        self.min_area_label = tk.Label(self.root, text="Minimum Detection Area")
        self.min_area_label.grid(row=8, column=self.video_width+1, padx=self.padx, pady=self.pady)

        self.min_area_input = tk.Entry(self.root, justify="right", width=18)
        self.min_area_input.insert(tk.END, "20")
        self.min_area_input.grid(row=8, column=self.video_width+2, padx=self.padx, pady=self.pady)

        # manual control
        self.tracking_label = tk.Label(self.root, text="Manual Control", font='Helvetica 18 bold')
        self.tracking_label.grid(row=10, column=self.video_width+1, padx=self.padx, pady=self.pady)

        self.up = tk.Button(self.root, text="Up", command=lambda: self.motor_command("elv", True))
        self.up.grid(row=11, column=self.video_width+1, padx=self.padx, pady=self.pady)

        self.down = tk.Button(self.root, text="Down", command=lambda: self.motor_command("elv", False))
        self.down.grid(row=12, column=self.video_width+1, padx=self.padx, pady=self.pady)

        self.clockwise = tk.Button(self.root, text="Clockwise", command=lambda: self.motor_command("az", False))
        self.clockwise.grid(row=11, column=self.video_width+2, padx=self.padx, pady=self.pady)

        self.anticlockwise = tk.Button(self.root, text="Anticlockwise", command=lambda: self.motor_command("az", True))
        self.anticlockwise.grid(row=12, column=self.video_width+2, padx=self.padx, pady=self.pady)

        self.steps_label = tk.Label(self.root, text="Enter Steps (>=0)")
        self.steps_label.grid(row=13, column=self.video_width+1, padx=self.padx, pady=self.pady)

        self.steps_entry = tk.Entry(self.root, justify="right", width=18)
        self.steps_entry.insert(tk.END, "0")
        self.steps_entry.grid(row=13, column=self.video_width+2, padx=self.padx, pady=self.pady)

        self.root.wm_title("Tracking Control Panel")
        self.root.wm_protocol("WM_DELETE_WINDOW", self.on_close)

    def draw_overlay(self, loop_time):
        # get frame dimensions
        (y, x) = self.show_frame.shape[:2]

        # centre circle in the guidescope
        cv2.circle(self.show_frame, (x//2, y//2), 5, (0, 0, 255), -1)

        # outline main camera FOV in the guidescope
        # main FOV 0.3056deg x 0.2196deg
        # guide FOV 1.91deg x 1.07deg
        # thus the main FOV is bounded by the centre 20.5% x 16% of the guide frame
        fov_x = MAINCAM_FOV_ARCSEC[0] / GUIDESCOPE_FOV_ARCSEC[0]
        fov_y = MAINCAM_FOV_ARCSEC[1] / GUIDESCOPE_FOV_ARCSEC[1]
        (w, h) = (int(fov_x*x), int(fov_y*y))
        cv2.rectangle(self.show_frame, ((x-w)//2, (y-h)//2), ((x+w)//2, (y+h)//2), (0, 0, 255), 1)

        # write tracking id on frame
        if self.tracker.tracking:
            self.show_frame = cv2.putText(self.show_frame, f"Tracking", 
                                        (5, y-25), 
                                        cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)
        self.show_frame = cv2.putText(self.show_frame, f"Tracking id: {self.tracker.target_id}", 
                                    (5, y-10), 
                                    cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)

        for id, obj in self.tracker.objects.items():
            x, y, width, height, dx, dy, mem = obj

            # if this is the object we are tracking
            # then change the colour from blue to red
            # else leave it as blue
            if id == self.tracker.target_id and self.tracker.tracking:
                self.tracker.target_pos = (x, y)
                text_colour = (0, 0, 255)
                box_colour = (0, 0, 255)
            else:
                text_colour = (255, 0, 0)
                box_colour = (0, 255, 0)

            if not width*height == 0:
                cv2.putText(self.show_frame, str(id), (x, y - 5), cv2.FONT_HERSHEY_PLAIN, 1, text_colour, 2)
                cv2.rectangle(self.show_frame, (x, y), (x + width, y + height), box_colour, 1)

        # update fps counter
        loop_time = time.perf_counter() - loop_time
        cv2.putText(self.show_frame, f"{1/loop_time:02.2f} FPS", (5, 15), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)

        if self.recording:
            # write 'recording' in bottom left of frame to notify user the video feed is being recorded
            cv2.putText(self.show_frame, f"Recording", (5, 30), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)

    def update_video(self, guide_frame, main_frame, loop_time):
        # save raw input image
        # if enabled, add the overlay
        self.main_frame = main_frame
        self.guide_frame = guide_frame

        if self.source_id:
            self.show_frame = self.main_frame
        elif not self.source_id:
            self.show_frame = self.guide_frame
            if self.overlay:
                self.draw_overlay(loop_time)

        # downsize resolution of video
        # OpenCV represents images in BGR order; however PIL
        # represents images in RGB order, so we need to swap
        # the channels, then convert to PIL and ImageTk format
        image = cv2.resize(self.show_frame, (1024, 576))
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = Image.fromarray(image)
        image = ImageTk.PhotoImage(image)

        # if the panel is not None, we need to initialize it
        if self.video_frame is None:
            print("Initialise GUI")
            self.video_frame = tk.Label(image=image)
            self.video_frame.image = image
            self.video_frame.grid(row=0, column=0, rowspan=32, columnspan=self.video_width, padx=self.padx, pady=self.pady)

        # otherwise, simply update the panel
        else:
            self.video_frame.configure(image=image)
            self.video_frame.image = image
    
    def update_tracking_id(self):
        try:
            self.tracker.target_id = int(self.tracking_id_input.get())
        except ValueError:
            self.tracker.target_id = 0

    def on_close(self):
        print("Exiting...")
        self.open = not self.open
        self.root.destroy()
        self.root.quit()

# camera object
class Camera():
    def __init__(self, dimensions, name, test=False):
        self.id = 0
        self.dimensions = dimensions

        codec = cv2.VideoWriter_fourcc(*'WMV3')
        fps = 20

        if test:
            self.vcap = cv2.VideoCapture("guidescope_test_0.mp4", cv2.CAP_ANY)
        elif not test:
            self.vcap = cv2.VideoCapture(self.id, cv2.CAP_ANY)
        self.vcap.set(cv2.CAP_PROP_FRAME_WIDTH, self.dimensions[0])
        self.vcap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.dimensions[1])
        self.vcap.set(cv2.CAP_PROP_FOURCC, codec)
    #    self.ret = False
    #    self.frame = None

        self.recording = False
        self.vwriter = cv2.VideoWriter(f"{name}_out.wmv", codec, fps, self.dimensions)

        print(f"{name} open")

    #def update_frame(self):
    #    self.ret, self.frame = self.vcap.read()

# sends a movement command to a given motor
# movement based on given steps
def move_motor(motor, steps):
    # commands are enclosed with <>
    # a/b for elevation/azimuth motor respectively
    # +/- for anticlockwise/clockwise, up/down respectively
    # positive integer for number of steps
    # e.g. <a+1000>; azimuth, clockwise, 1000 steps

    cmd = "<"

    if motor == "az":
        cmd += "b"
    elif motor == "elv":
        cmd += "a"
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

    # commands must be encoded using utf-8
    # command is then sent to the arduino

    cmd = cmd.encode("utf-8")
    ser.write(cmd)

# detects objects in the frame
# returns a list of object coordinates and sizes
def get_bounding_rectangles(frame, min_area, threshold_val):
    #greyscale and blur both the current and previous frame
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    frame = cv2.GaussianBlur(frame, (11,11), 0)

    #take threshold of frame
    #dilate and erode to join fragmented objects
    #get a list of edges and their positions
    _, threshold = cv2.threshold(frame, threshold_val, 255, cv2.THRESH_BINARY)
    dilated = cv2.dilate(threshold, None, iterations = 1)
    eroded = cv2.erode(dilated, None, iterations = 1)
    edges, _ = cv2.findContours(eroded, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    #initialise an empty list to hold object bounding rectangles, (x, y, width, height)
    object_bounding_rectangles = []

    #loop through list of valid objects and store their positions
    for edge in edges:
        if min_area < cv2.contourArea(edge):
            object_bounding_rectangles.append(cv2.boundingRect(edge))

    return object_bounding_rectangles


def main():

    # main setup
    print("Starting main...")

    guidescope = Camera(GUIDESCOPE_RESOLUTION, "guidescope", test=True)
    maincam = Camera(MAINCAM_RESOLUTION, "maincam")

    #CALIBRATE

    #GO TO ISS

    # initialise euclidean distance tracker object
    tracker = Tracker()

    # GUI setup
    gui = Gui(tracker)

    # main tracking loop
    # run until user closes GUI
    while gui.open:
        # start timer for fps
        loop_time = time.perf_counter()

        # get new frame
        ret, guide_frame = guidescope.vcap.read()
        guide_frame = cv2.rotate(guide_frame, cv2.ROTATE_180)

        # skip loop if there is no new frame;
        if not ret:
            continue

        _ , main_frame = maincam.vcap.read()


        # get a list of bounding rectangles for each object in frame
        object_bounding_rectangles = get_bounding_rectangles(guide_frame, int(gui.min_area_input.get()), int(gui.min_threshold_input.get()))

        # get a list of objects with ids
        tracker.update(object_bounding_rectangles)

        # update tracking id
        gui.update_tracking_id()

        # move the motors if we are tracking an object
        if tracker.tracking:
            x, y = tracker.target_pos
            x_mov = guidescope.dimensions[0]/2 - x
            y_mov = guidescope.dimensions[1]/2 - y
            #print(f"{x_mov}, {y_mov}")

        # send video to gui
        gui.update_video(guide_frame, main_frame, loop_time)
        gui.root.update()

    # close serial connection
    ser.close()

    print("Done.")