#!/usr/bin/env python3

from argparse import ArgumentParser
import collections
import os
from tkinter.constants import LEFT, RIGHT
import rclpy
from rclpy.node import Node
import tkinter as tk 
import threading
import time
import subprocess
import psutil
from tkinter import Canvas, Frame, messagebox
import numpy as np
import cv2
from cv_bridge import CvBridge
from PIL import Image, ImageTk
from sensor_msgs.msg import Image as ROSImage
from my_robot_interfaces.msg import MotorControlData

from .april_tag import Detector
from .april_tag import _get_dll_path
from .april_tag import detect_tags

import math

# SSD CNN Constants
SSD_INPUT_SIZE = 320

RESIZED_DIMENSIONS = (300, 300) # Dimensions that SSD was trained on. 
IMG_NORM_RATIO = 0.007843 # In grayscale a pixel can range between 0 and 255


#------------------------------------------------------------
# class DesktopUI
#------------------------------------------------------------
class DesktopUI(tk.Tk):

    def __init__(self, node):
        super().__init__()

        self.desktop_ui_node = node

        # Create the main window
        self.title("Desktop UI")

        self.geometry("1536x864")
        #self.geometry("1280x720")
        self.resizable(False, False)

        # set the position of the window to the center of the screen        
        #self.eval('tk::PlaceWindow . center')

        self.configure(background="wheat3")

        # Adds an icon to the app in the Activities bar
        # icon_img = tk.Image("photo", file = "/home/ubuntu/ros2_ws/src/pumpkin_bot_bringup/launch/pumpkin.png")
        # self.tk.call('wm', 'iconphoto', self._w, icon_img)

        self.columnconfigure(0, weight=1)
        self.columnconfigure(1, weight=3)
        self.columnconfigure(2, weight=1)

        left_frame = Frame(self, borderwidth=3, relief=tk.SUNKEN, width=300, height=600)
        left_frame.pack_propagate(0)
        left_frame.grid(row=0, column=0, pady=(10,10))

        self.status_label = tk.Label(self, bg="black", fg="orange", text="                                     ", font=("Arial", 22), takefocus=0)
        self.status_label.grid(row=1, column=1, pady=(25,0))
        self.status_label2 = tk.Label(self, bg="black", fg="orange", text="                                     ", font=("Arial", 22), takefocus=0)
        self.status_label2.grid(row=2, column=1, pady=(25,0))
        
        right_frame = Frame(self, borderwidth=3, relief=tk.SUNKEN, width=300, height=600)
        right_frame.pack_propagate(0)
        right_frame.grid(row=0, column=2, pady=(10,10))

        self.lane_detection_button = tk.Button(left_frame, text = "Lane Detection", takefocus=0, width=100, 
                command=self.lane_detection_button_pressed)
        self.lane_detection_button.pack(anchor="center", padx=(10,10), pady=(10,10))

        self.apriltag_detection_button = tk.Button(left_frame, text = "AprilTag Detection", takefocus=0, width=100, 
                command=self.apriltag_detection_button_pressed)
        self.apriltag_detection_button.pack(anchor="center", padx=(10,10), pady=(10,10))

        quit_button = tk.Button(left_frame, text = "Quit", takefocus=0, width=100, command=self.quit_button_callback)
        quit_button.pack(anchor="center", padx=(10,10), pady=(10,10))

        self.protocol("WM_DELETE_WINDOW", self.quit_button_callback)

        stop_button = tk.Button(right_frame, text = "Stop", takefocus=0, width=100, command=self.stop_button_callback)
        stop_button.pack(anchor="center", padx=(10,10), pady=(10,10))

        forward_button = tk.Button(right_frame, text = "Forward", takefocus=0, width=100, command=self.forward_button_callback)
        forward_button.pack(anchor="center", padx=(10,10), pady=(10,10))

        #self.statusLabel.config(text="Status: Waiting for Initialization Command...")

        self.started_lane_detection = False
        self.started_apriltag_detection = False

        self.lane_detection_cancel_id = 0
        self.apriltag_detection_cancel_id = 0

        self.ros_frame = np.zeros((640, 480, 3), np.uint8)

        self.lane_detection_image = self.ros_frame
        self.apriltag_detection_image = self.ros_frame

        self.open_cv_canvas = Canvas(self, width=600, height=480, takefocus=0)  
        self.open_cv_canvas.grid(row=0, column=1, pady=(10,0))

    def quit_button_callback(self):
        if messagebox.askokcancel("Quit", "Do you want to quit?"):

            self.status_label.config(text="Status: Quitting application...")

            cv2.destroyAllWindows()

            self.destroy()  

    def stop_button_callback(self):

        msg = MotorControlData()

        msg.front_right_power = 0.0
        msg.front_left_power = 0.0
        msg.rear_right_power = 0.0
        msg.rear_left_power = 0.0

        self.desktop_ui_node.motor_control_data_msg = msg

    def forward_button_callback(self):
        
        msg = MotorControlData()

        msg.front_right_power = 0.2
        msg.front_left_power = 0.2
        msg.rear_right_power = 0.2
        msg.rear_left_power = 0.2

        self.desktop_ui_node.motor_control_data_msg = msg


    def start_main_loop(self):
        self.mainloop() 

    #------------------------------------------------------------
    # Lane Detection Methods
    #------------------------------------------------------------

    def draw_the_lines(self, image, lines):
        # create a distinct image for the lines [0,255] - all 0 values means black image
        lines_image = np.zeros((image.shape[0], image.shape[1], 3), dtype=np.uint8)

        # there are (x,y) for the starting and end points of the lines
        if lines is not None:
            for line in lines:
                for x1, y1, x2, y2 in line:
                    cv2.line(lines_image, (x1, y1), (x2, y2), (255, 0, 0), thickness=3)

        # finally we have to merge the image with the lines
        image_with_lines = cv2.addWeighted(image, 0.8, lines_image, 1, 0.0)

        return image_with_lines


    def region_of_interest(self, image, region_points):
        # we are going to replace pixels with 0 (black) - the regions we are not interested
        mask = np.zeros_like(image)
        # the region that we are interested in is the lower triangle - 255 white pixels
        cv2.fillPoly(mask, region_points, 255)
        # we have to use the mask: we want to keep the regions of the original image where  
        # the mask has white colored pixels
        masked_image = cv2.bitwise_and(image, mask)
        return masked_image


    def get_detected_lanes(self, image):

        image = self.ros_frame

        (height, width) = (image.shape[0], image.shape[1])

        # we have to turn the image into grayscale
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # edge detection kernel (Canny's algorithm)
        canny_image = cv2.Canny(gray_image, 100, 120)

        # we are interested in the "lower region" of the image (there are the driving lanes)
        region_of_interest_vertices = [
            (0, height),
            (width / 2, height * 0.65),
            (width, height)
        ]

        # we can get rid of the un-relevant part of the image
        # we just keep the lower triangle region
        cropped_image = self.region_of_interest(
            canny_image, np.array([region_of_interest_vertices], np.int32))

        # use the line detection algorithm (radians instead of degrees 1 degree = pi / 180)
        lines = cv2.HoughLinesP(cropped_image, rho=2, theta=np.pi / 180, threshold=50, lines=np.array([]),
                            minLineLength=40, maxLineGap=150)

        # draw the lines on the image
        image_with_lines = self.draw_the_lines(image, lines)

        return image_with_lines
    
 
    def start_lane_detection(self):
 
        self.lane_detection_image = self.get_detected_lanes(self.lane_detection_image)

        self.lane_detection_image = Image.fromarray(self.lane_detection_image) # to PIL format
        self.lane_detection_image = ImageTk.PhotoImage(self.lane_detection_image) # to ImageTk format

        # Update image
        self.open_cv_canvas.create_image(0, 0, anchor=tk.NW, image=self.lane_detection_image)

        # Repeat every 'interval' ms
        self.lane_detection_cancel_id = self.after(10, self.start_lane_detection)


    def lane_detection_button_pressed(self):
        if self.started_lane_detection:
            self.started_lane_detection = False
            self.lane_detection_button['text'] = 'Lane Detection'
            self.open_cv_canvas.delete("all")
            self.after_cancel(self.lane_detection_cancel_id)
        elif not self.started_face_detection:
            self.open_cv_canvas.delete("all")
            self.started_lane_detection = True
            self.lane_detection_button['text'] = 'Lane Detection*'
            self.start_lane_detection()

    #------------------------------------------------------------
    # AprilTag Detection Methods
    #------------------------------------------------------------
    def get_detected_apriltags(self, image):
        
        image = self.ros_frame

        # |fx  0  cx|
        # |0  fy  cy|
        # |0   0   1|

        # TODO Get camera config from the config file in /camera_config/....
        result, overlay = detect_tags(image,
                                     self.apriltag_detector,
                                     #camera_params=(3156.71852, 3129.52243, 359.097908, 239.736909),
                                     camera_params=(622.27892, 622.97536, 333.70651, 211.43233),
                                     tag_size=0.1651,
                                     vizualization=3,
                                     verbose=0,
                                     annotation=True
                                    )
        if result:                            
            # print(result[0].tostring(collections.OrderedDict([('Pose', result[1]),
            #                                                   ('InitError', result[2]),
            #                                                   ('FinalError', result[3])]),
            #                                                   indent=2))
            #print(result[0].tostring())
            print('Tag ID: {}'.format(result[0].tag_id))
            # print(result[0].tostring(collections.OrderedDict([('Pose', result[1])]),
            #                                                     indent=1))
            #print(result[1])
            r11 = result[1][0][0]
            r21 = result[1][1][0]
            r31 = result[1][2][0]
            r32 = result[1][2][1]
            r33 = result[1][2][2]

            yaw = "{:.2f}".format(np.degrees(np.arctan2(r21, r11)))
            pitch = "{:.2f}".format(np.degrees(np.arctan2(-r31, math.sqrt(r32**2 + r33**2))))
            roll = "{:.2f}".format(np.degrees(np.arctan2(r32, r33)))

            tx = "{:.4f}".format(result[1][0][3])
            ty = "{:.4f}".format(result[1][1][3])
            tz = "{:.4f}".format(result[1][2][3])
            print(f'TX: {tx} TY: {ty} TZ: {tz}')
            print(f'Yaw: {yaw} Pitch: {pitch} Roll: {roll}')
            print("=================================================")
            self.status_label.config(text=f'TX: {tx} TY: {ty} TZ: {tz}')
            self.status_label2.config(text=f'Y: {yaw} P: {pitch} R: {roll}')

        return overlay

    def start_apriltag_detection(self):

        self.apriltag_detection_image = self.get_detected_apriltags(self.apriltag_detection_image)

        self.apriltag_detection_image = Image.fromarray(self.apriltag_detection_image) # to PIL format
        self.apriltag_detection_image = ImageTk.PhotoImage(self.apriltag_detection_image) # to ImageTk format
        # Update image
        self.open_cv_canvas.create_image(0, 0, anchor=tk.NW, image=self.apriltag_detection_image)
        # Repeat every 'interval' ms
        self.apriltag_detection_cancel_id = self.after(10, self.start_apriltag_detection)

    def apriltag_detection_button_pressed(self):
        if self.started_apriltag_detection:
            self.started_apriltag_detection = False
            self.apriltag_detection_button['text'] = 'AprilTag Detection'
            self.open_cv_canvas.delete("all")
            self.after_cancel(self.apriltag_detection_cancel_id)
        elif not self.started_lane_detection:
            self.open_cv_canvas.delete("all")
            self.apriltag_detector = Detector(searchpath=_get_dll_path())
            self.started_apriltag_detection = True
            self.apriltag_detection_button['text'] = 'AprilTag Detection*'
            self.start_apriltag_detection()

#------------------------------------------------------------
# class DesktopUserInterfaceNode
#------------------------------------------------------------
class DesktopUserInterfaceNode(Node):
    
    def __init__(self):
        super().__init__("desktop_user_interface") 

        self.user_interface = DesktopUI(self)

        self.motor_control_data_msg = MotorControlData()
        self.motor_control_data_msg.front_right_power = 0.0
        self.motor_control_data_msg.front_left_power = 0.0
        self.motor_control_data_msg.rear_right_power = 0.0
        self.motor_control_data_msg.rear_left_power = 0.0

        self.motor_control_data_publisher = self.create_publisher(MotorControlData, '/amr/motor_control', 10)
        self.motor_control_data_timer = self.create_timer(0.2, self.publish_motor_control_data)

        # Create the subscriber. This subscriber will receive an Image
        # from the video_frames topic. The queue size is 10 messages.
        self.video_frames_subscription = self.create_subscription(
            ROSImage, 
            'image', # from pi using cam_2_image
            #'image_raw', # when using usb_cam
            self.video_frames_listener_callback, 
            10)
        self.video_frames_subscription # prevent unused variable warning
      
        # Used to convert between ROS and OpenCV images
        self.cv_bridge = CvBridge()

        # Start spinning
        spinning_thread = threading.Thread(target=self.start_spinning)
        spinning_thread.start()

        self.user_interface.start_main_loop()

    def publish_motor_control_data(self):
        self.motor_control_data_publisher.publish(self.motor_control_data_msg)
  
    def video_frames_listener_callback(self, data):
        """
        Callback function for video frame processing.
        """
        # Display the message on the console
        #self.get_logger().info('Receiving video frame')
 
        # Convert ROS Image message to OpenCV image and display it
        current_frame = self.cv_bridge.imgmsg_to_cv2(data)
        self.user_interface.ros_frame = current_frame
    
        # Display image
        #cv2.imshow("camera", current_frame)
        #cv2.waitKey(1)

    def start_spinning(self):
        rclpy.spin(self)
        rclpy.shutdown()

#------------------------------------------------------------
# main
#------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = DesktopUserInterfaceNode()

#------------------------------------------------------------
# Entry Point
#------------------------------------------------------------
if __name__ == "__main__":
    main()
