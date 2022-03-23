from itertools import count
from datetime import datetime
from PIL import Image, ImageTk
import numpy as np
import tkinter as tk
import serial
import time

import os
os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"

import cv2
from scipy.optimize import linear_sum_assignment

# global constants

# setup options
TEST_VIDEO = True
TIMERS = True
ARDUINO_COM = 3
GUIDESCOPE_ID = 1
MAINCAM_ID = 0

# motor specs
STEP_SIZE_ARCSEC = 6480
GEAR_RATIO = 50
STEP_SIZE_ARCSEC /= GEAR_RATIO

# camera specs
GUIDESCOPE_FOV_ARCSEC = (6876, 3852)
GUIDESCOPE_RESOLUTION = (1920, 1080)
GUIDESCOPE_FPS = 20

MAINCAM_FOV_ARCSEC = (1100.16, 790.56)
MAINCAM_RESOLUTION = (3856, 2764)
MAINCAM_FPS = 7

# serial communications
BAUD_RATE = 9600

# tracking constants
# factor by which to downscale the guidescope frame for image processing
# must be a common factor of both guidescope dimensions
SIZE_RATIO = 4

# tracker object
# tracks objects across frames
class Tracker:
    def __init__(self):
        #store info on objects in a dictionary
        #id : x, y, width, height, dx, dy, memory
        self.previous_objects = {}
        self.objects = {}
        self.active = False

        # info for the target object
        self.target_tracking = False
        self.target_id = 57
        self.target_pos = (0, 0)
        self.command_timer = time.perf_counter()

    # toggle the tracking param
    # when True, the motors will follow the target object
    def track(self):
        self.target_tracking = not self.target_tracking

    # assign ids to the objects in a new frame
    def update(self, detection):
        if self.active:
            #number of frames missing objects are remembered for
            MEMORY = 10
            #an artificial cost to add to the list 
            #matches with missing objects
            THRESHOLD = 200

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
        elif not self.active:
            self.clear()

    def switch(self):
        self.active = not self.active
        if not self.active:
            print("Object Tracking Off")
            self.clear
        else:
            print("Object Tracking On")

    # clear the memory of the tracker
    def clear(self):
        self.objects = {}
        self.previous_objects = {}

# gui object
# handles GUI
class Gui():

    def overlay_command(self):
        self.overlay = not self.overlay
    
    def record_command(self, camera):
        camera.recording = not camera.recording
        if camera.recording:
            camera.start_video()
        elif not camera.recording:
            camera.stop_video()
    
    def screenshot_command(self, camera):
        file_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S-%f")
        cv2.imwrite(f".\Screenshots\{camera.name}_{file_time}.png", camera.frame)
        print(f"Saving screenchot as \"{camera.name}_{file_time}.png\"")
    
    def motor_command(self, motor, dir):
        steps = abs(int(self.steps_entry.get()))
        if not dir:
            steps *= -1
        try:
            move_motor(motor, steps, self.ser)
            print(f"Moving {motor} motor {steps} steps")
        except ValueError as e:
            print(e)
    
    def source_command(self):
        self.source_id = not self.source_id

    def __init__(self, tracker, guidescope, maincam):
        self.tracker = tracker
        self.ser = None

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
        video_options_row = 0

        self.video_label = tk.Label(self.root, text="Video Options", font='Helvetica 18 bold')
        self.video_label.grid(row=video_options_row, column=self.video_width+1, padx=self.padx, pady=self.pady)

        self.overlay = True
        self.overlay_button = tk.Button(self.root, text="Toggle Overlay", command=self.overlay_command)
        self.overlay_button.grid(row=video_options_row+1, column=self.video_width+1, padx=self.padx, pady=self.pady)

        self.source_id = False
        self.source_button = tk.Button(self.root, text="Toggle Source", command=self.source_command)
        self.source_button.grid(row=video_options_row+1, column=self.video_width+2, padx=self.padx, pady=self.pady)

        self.tracking_id_label = tk.Label(self.root, text="Guidescope Actions")
        self.tracking_id_label.grid(row=video_options_row+2, column=self.video_width+1, padx=self.padx, pady=self.pady)

        self.recording = False
        self.record_button = tk.Button(self.root, text="Toggle Recording", command=lambda: self.record_command(guidescope))
        self.record_button.grid(row=video_options_row+3, column=self.video_width+1, padx=self.padx, pady=self.pady)

        self.screenshot = False
        self.screenshot_button = tk.Button(self.root, text="Take Screenshot", command=lambda: self.screenshot_command(guidescope))
        self.screenshot_button.grid(row=video_options_row+3, column=self.video_width+2, padx=self.padx, pady=self.pady)

        self.tracking_id_label = tk.Label(self.root, text="Main Camera Actions")
        self.tracking_id_label.grid(row=video_options_row+4, column=self.video_width+1, padx=self.padx, pady=self.pady)

        self.recording = False
        self.record_button = tk.Button(self.root, text="Toggle Recording", command=lambda: self.record_command(maincam))
        self.record_button.grid(row=video_options_row+5, column=self.video_width+1, padx=self.padx, pady=self.pady)

        self.screenshot = False
        self.screenshot_button = tk.Button(self.root, text="Take Screenshot", command=lambda: self.screenshot_command(maincam))
        self.screenshot_button.grid(row=video_options_row+5, column=self.video_width+2, padx=self.padx, pady=self.pady)

        # tracking options
        tracking_options_row = 7

        self.tracking_label = tk.Label(self.root, text="Tracking Options", font='Helvetica 18 bold')
        self.tracking_label.grid(row=tracking_options_row, column=self.video_width+1, padx=self.padx, pady=self.pady)

        self.clear_button = tk.Button(self.root, text="Toggle Object Tracking", command=self.tracker.switch)
        self.clear_button.grid(row=tracking_options_row+1, column=self.video_width+2, padx=self.padx, pady=self.pady)

        self.tracking = False
        self.tracking_button = tk.Button(self.root, text="Toggle Target Tracking", command=self.tracker.track)
        self.tracking_button.grid(row=tracking_options_row+1, column=self.video_width+1, padx=self.padx, pady=self.pady)

        self.tracking_id_label = tk.Label(self.root, text="Tracking ID")
        self.tracking_id_label.grid(row=tracking_options_row+2, column=self.video_width+1, padx=self.padx, pady=self.pady)

        self.tracking_id_input = tk.Entry(self.root, justify="right", width=18)
        self.tracking_id_input.insert(tk.END, "0")
        self.tracking_id_input.grid(row=tracking_options_row+2, column=self.video_width+2, padx=self.padx, pady=self.pady)

        self.min_threshold_label = tk.Label(self.root, text="Minimum Threshold Value")
        self.min_threshold_label.grid(row=tracking_options_row+3, column=self.video_width+1, padx=self.padx, pady=self.pady)

        self.min_threshold_input = tk.Entry(self.root, justify="right", width=18)
        self.min_threshold_input.insert(tk.END, "20")
        self.min_threshold_input.grid(row=tracking_options_row+3, column=self.video_width+2, padx=self.padx, pady=self.pady)

        self.min_area_label = tk.Label(self.root, text="Minimum Detection Area")
        self.min_area_label.grid(row=tracking_options_row+4, column=self.video_width+1, padx=self.padx, pady=self.pady)

        self.min_area_input = tk.Entry(self.root, justify="right", width=18)
        self.min_area_input.insert(tk.END, "20")
        self.min_area_input.grid(row=tracking_options_row+4, column=self.video_width+2, padx=self.padx, pady=self.pady)

        # manual control
        manual_control_row = 12

        self.tracking_label = tk.Label(self.root, text="Manual Control", font='Helvetica 18 bold')
        self.tracking_label.grid(row=manual_control_row, column=self.video_width+1, padx=self.padx, pady=self.pady)

        self.up = tk.Button(self.root, text="Up", command=lambda: self.motor_command("elv", True))
        self.up.grid(row=manual_control_row+1, column=self.video_width+1, padx=self.padx, pady=self.pady)

        self.down = tk.Button(self.root, text="Down", command=lambda: self.motor_command("elv", False))
        self.down.grid(row=manual_control_row+2, column=self.video_width+1, padx=self.padx, pady=self.pady)

        self.clockwise = tk.Button(self.root, text="Clockwise", command=lambda: self.motor_command("az", False))
        self.clockwise.grid(row=manual_control_row+1, column=self.video_width+2, padx=self.padx, pady=self.pady)

        self.anticlockwise = tk.Button(self.root, text="Anticlockwise", command=lambda: self.motor_command("az", True))
        self.anticlockwise.grid(row=manual_control_row+2, column=self.video_width+2, padx=self.padx, pady=self.pady)

        self.steps_label = tk.Label(self.root, text="Enter Steps (>=0)")
        self.steps_label.grid(row=manual_control_row+3, column=self.video_width+1, padx=self.padx, pady=self.pady)

        self.steps_entry = tk.Entry(self.root, justify="right", width=18)
        self.steps_entry.insert(tk.END, "0")
        self.steps_entry.grid(row=manual_control_row+3, column=self.video_width+2, padx=self.padx, pady=self.pady)

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
        if self.tracker.target_tracking:
            self.show_frame = cv2.putText(self.show_frame, f"Tracking", 
                                        (5, y-25), 
                                        cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)
        self.show_frame = cv2.putText(self.show_frame, f"Tracking id: {self.tracker.target_id}", 
                                    (5, y-10), 
                                    cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)

        if self.tracker.active:
            for id, obj in self.tracker.objects.items():
                x, y, width, height, dx, dy, mem = obj

                # if this is the object we are tracking
                # then change the colour from blue to red
                # else leave it as blue
                if id == self.tracker.target_id and self.tracker.target_tracking:
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
        image = cv2.resize(self.show_frame, (768, 432))
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
    def __init__(self, dimensions, name, id, test=False):
        self.name = name
        self.id = id
        self.dimensions = dimensions

        self.codec = cv2.VideoWriter_fourcc(*'WMV3')
        self.fps = 20

        if test:
            self.vcap = cv2.VideoCapture("guidescope_test_0.mp4", cv2.CAP_ANY)
        elif not test:
            self.vcap = cv2.VideoCapture(self.id, cv2.CAP_ANY)
        self.vcap.set(cv2.CAP_PROP_FRAME_WIDTH, self.dimensions[0])
        self.vcap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.dimensions[1])
        self.vcap.set(cv2.CAP_PROP_FOURCC, self.codec)
        self.ret = False
        self.frame = None

        self.recording = False
        self.vwriter = None

        print(f"{name} open")

    def update_frame(self):
        if self.vcap.grab():
            self.ret, self.frame = self.vcap.retrieve()
        else:
            print(f"{self.name} disconnected")
            self.ret = False

    def start_video(self):
        self.recording = True
        self.vwriter = cv2.VideoWriter(f"{self.name}_out.wmv", self.codec, self.fps, GUIDESCOPE_RESOLUTION)
        print(f"Recording {self.name}.")

    def stop_video(self):
        self.recording = False
        print(f"Saved {self.name}_out.wmv")
        self.vwriter.release()

    def write_frame(self):
        if self.recording:
            out = cv2.resize(self.frame, GUIDESCOPE_RESOLUTION)
            self.vwriter.write(out)

# sends a movement command to a given motor
# movement based on given steps
def move_motor(motor, steps, ser, _count=count(1)):
    # commands are enclosed with <>
    # a/b for elevation/azimuth motor respectively
    # +/- for anticlockwise/clockwise, up/down respectively
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

    # commands must be encoded using utf-8
    # command is then sent to the arduino

    cmd = cmd.encode("utf-8")
    ser.write(cmd)

    return next(_count)

# detects objects in the frame
# returns a list of object coordinates and sizes
def get_bounding_rectangles(frame, min_area, threshold_val):
    # check min area and threshold values
    try:
        min_area = int(min_area)
    except ValueError:
        min_area = 20

    try:
        threshold_val = int(threshold_val)
    except ValueError:
        threshold_val = 20

    # reduce the resolution of the frame before doing image processing
    frame = cv2.resize(frame, (np.array(GUIDESCOPE_RESOLUTION)/SIZE_RATIO).astype(np.int))

    #greyscale and blur both the current and previous frame
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #frame = cv2.GaussianBlur(frame, (11,11), 0)

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
            object_bounding_rectangles.append(np.array(cv2.boundingRect(edge))*SIZE_RATIO)

    return object_bounding_rectangles

# open serial comms
def open_serial(gui):
    ser = serial.Serial(f"COM{ARDUINO_COM}", BAUD_RATE)
    ser.timeout = 1
    if not ser.isOpen():
        ser.open()
        time.sleep(0.5)
    gui.ser = ser
    return ser

def main():

    # main setup
    print("Starting main...")

    guidescope = Camera(GUIDESCOPE_RESOLUTION, "guidescope", GUIDESCOPE_ID, test=TEST_VIDEO)
    maincam = Camera(MAINCAM_RESOLUTION, "maincam", MAINCAM_ID)
    maincam_timer = time.perf_counter()

    #CALIBRATE

    #GO TO ISS

    # initialise euclidean distance tracker object
    tracker = Tracker()

    # GUI setup
    gui = Gui(tracker, guidescope, maincam)

    # open serial connection
    ser = open_serial(gui)

    # track number of commands sent
    i = 0

    # main tracking loop
    # run until user closes GUI
    while gui.open:
        # start timer for fps
        loop_time = time.perf_counter()

        # gui loop
        gui_update_time = time.perf_counter()
        gui.root.update()
        if TIMERS: print(f"GUI Time: {time.perf_counter()-gui_update_time}")

        # get new frames
        frame_time = time.perf_counter()
        guidescope.update_frame()
        #guidescope.frame = cv2.rotate(guidescope.frame, cv2.ROTATE_180)

        # use a timer to stop loop from hanging if there is no new maincam frame
        if time.perf_counter() - maincam_timer > 1/MAINCAM_FPS:
            maincam.update_frame()
            maincam.frame = cv2.rotate(maincam.frame, cv2.ROTATE_180)
            maincam_timer = time.perf_counter()
        if TIMERS: print(f"New Frames: {time.perf_counter() - frame_time}")

        # exit loop if frames arent recieved
        if not (guidescope.ret and maincam.ret):
            break

        if tracker.active:
            # get a list of bounding rectangles for each object in frame
            cv_time = time.perf_counter()
            object_bounding_rectangles = get_bounding_rectangles(guidescope.frame, gui.min_area_input.get(), gui.min_threshold_input.get())
            if TIMERS: print(f"Image Processing: {time.perf_counter()-cv_time}")

            tracking_time = time.perf_counter()
            # get a list of objects with ids
            tracker.update(object_bounding_rectangles)

            # update tracking id
            gui.update_tracking_id()

            # move the motors if we are tracking an object
            # will only execute every .5 seconds max
            # sending commands too quickly seems to overwhelm the serial connection
            if tracker.target_tracking and time.perf_counter() - tracker.command_timer > 1:

                # hacky fix
                # serial comms seem to stop working at about ~256 commands
                # so restart connection before then
                # counter i tracks commands sent
                if int(i/2) % 120 == 0:
                    print("reset serial connection")
                    ser.close()
                    ser = open_serial(gui)

                # calculate steps to take
                # caclulates distance to centre in pixels, then converts to steps
                x, y = tracker.target_pos
                x_mov = guidescope.dimensions[0]/2 - x
                y_mov = guidescope.dimensions[1]/2 - y
                x_mov *= GUIDESCOPE_FOV_ARCSEC[0] / STEP_SIZE_ARCSEC / guidescope.dimensions[0]
                y_mov *= GUIDESCOPE_FOV_ARCSEC[1] / STEP_SIZE_ARCSEC / guidescope.dimensions[1]

                # take steps
                # move_motor returns the number of commands sent since the start
                i = move_motor("az", x_mov, ser)
                i = move_motor("elv", y_mov, ser)

                # update timer
                tracker.command_timer = time.perf_counter()

                #print(f"{x_mov}, {y_mov}")
            if TIMERS: print(f"Tracking: {time.perf_counter() - tracking_time}")

        # send video to gui
        gui_update_time = time.perf_counter()
        gui.update_video(guidescope.frame, maincam.frame, loop_time)
        if TIMERS: print(f"Video Update: {time.perf_counter()-gui_update_time}")

        # gui loop
        gui_update_time = time.perf_counter()
        gui.root.update()
        if TIMERS: print(f"GUI Time: {time.perf_counter()-gui_update_time}")

        # record frames to video
        guidescope.write_frame()
        maincam.write_frame()

        if TIMERS: print(f"Total loop: {time.perf_counter() - loop_time}\n")

    # close serial connection
    ser.close()

    # close video writers
    if guidescope.recording:
        guidescope.vwriter.release()
    if maincam.recording:
        maincam.vwriter.release()

    # make sure GUI closes in the event of a camera disconnect
    if gui.open:
        gui.on_close()

    print("Done.")

if __name__ == "__main__":
    main()