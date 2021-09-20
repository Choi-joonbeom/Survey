# -*- coding:utf-8 -*-
#!/usr/bin/python

import cv2
import rospy
import string
import time
import itertools
import numpy as np
import math
import os
import rosbag
import datetime
import glob
import csv
import argparse
import time

from cv_bridge import CvBridge
import sensor_msgs.point_cloud2
import message_filters
from message_filters import TimeSynchronizer, ApproximateTimeSynchronizer, Subscriber
from sensor_receiver.msg import Can
from sensor_receiver.msg import CarInfo
from sensor_receiver.msg import Radar
from sensor_receiver.msg import Ultrasonic

from sensor_msgs.msg import NavSatFix, Imu, Image, PointCloud2

from std_msgs.msg import Int32
from std_msgs.msg import Header

import subprocess, yaml

parser = argparse.ArgumentParser()
parser.add_argument(
    "bagfile_path", help="directory path where your bag files are located"
)
parser.add_argument("dcfile_path", help="directory path where you want to save")
args = parser.parse_args()

bagfile_path = args.bagfile_path
dcfile_path = args.dcfile_path
index_num = 0
flag = None
fourcc = None
front_out = None
rear_out = None
left_out = None
# right_out = None


def get_size(start_path="."):
    total_size = 0
    for dirpath, dirnames, filenames in os.walk(start_path):
        for f in filenames:
            fp = os.path.join(dirpath, f)
            total_size += os.path.getsize(fp)
    return total_size


def mkdir():
    global output_dir
    global scenario_id
    try:
        if not os.path.exists(output_dir + scenario_id):
            os.makedirs(output_dir + scenario_id)
    except OSError:
        print("Error: Creating directory. " + output_dir + scenario_id)


def image_saver(msg, camera_type, index):
    global front_out
    global rear_out
    global left_out
    # global right_out

    # mkdir(camera_type+'/')
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(msg, "bgr8")
    # cv2.imwrite(os.path.join(output_dir + scenario_id + camera_type +"/", "%06i.jpg" % index), img)
    if camera_type == "front_camera":
        front_out.write(img)
    elif camera_type == "rear_camera":
        rear_out.write(img)
    elif camera_type == "left_camera":
        left_out.write(img)
    # elif camera_type == 'right_camera':
    # right_out.write(img)


def callback(front_msg, rear_msg, left_msg):
    global index_num
    global flag

    flag = True

    # No Sampling
    # Sampling = index_num % 10
    # if Sampling == 0:
    image_saver(front_msg, "front_camera", index_num)
    image_saver(rear_msg, "rear_camera", index_num)
    image_saver(left_msg, "left_camera", index_num)
    # image_saver(right_msg, 'right_camera', index_num)

    print(index_num)
    index_num += 1
    flag = False


def main():

    global pub_front
    global index_num
    global flag
    global output_dir
    global scenario_id
    global bagfile_path
    global dcfile_path
    global fourcc
    global front_out
    global rear_out
    global left_out
    # global right_out

    rospy.init_node("bag_to_dc")

    pub_front = rospy.Publisher("/front_cam/image_raw", Image, queue_size=1)
    pub_rear = rospy.Publisher("/rear_cam/image_raw", Image, queue_size=1)
    pub_left = rospy.Publisher("/left_cam/image_raw", Image, queue_size=1)
    # pub_right = rospy.Publisher("/right_cam/image_raw", Image, queue_size=1)

    front_msg = message_filters.Subscriber("/front_cam/image_raw", Image)
    rear_msg = message_filters.Subscriber("/rear_cam/image_raw", Image)
    left_msg = message_filters.Subscriber("/left_cam/image_raw", Image)
    # right_msg = message_filters.Subscriber('/right_cam/image_raw', Image)

    bagfile_list = bagfile_path
    file_list = os.listdir(bagfile_list)
    file_list_bag = [file for file in file_list if file.endswith(".bag")]
    file_list_bag.sort()
    # print(file_list_bag)
    bridge = CvBridge()

    flag = False

    for line in file_list_bag:
        index_num = 0

        # Run ApproximateTimeSynchronizer
        print("Run ApproximateTimeSynchronizer")
        ts = message_filters.ApproximateTimeSynchronizer(
            [front_msg, rear_msg, left_msg], 10, 0.1
        )
        ts.registerCallback(callback)

        bag_path = bagfile_path + line  # Path to ROS bag you want to repair

        output_dir = dcfile_path
        print(output_dir)
        scenario_id = line[:-4] + "/"
        mkdir()

        fourcc = cv2.VideoWriter_fourcc(*"DIVX")
        front_out = cv2.VideoWriter(
            output_dir + scenario_id + "front_camera.mp4", fourcc, 10.0, (1920, 1080)
        )
        rear_out = cv2.VideoWriter(
            output_dir + scenario_id + "rear_camera.mp4", fourcc, 10.0, (1920, 1080)
        )
        left_out = cv2.VideoWriter(
            output_dir + scenario_id + "left_camera.mp4", fourcc, 10.0, (1920, 1080)
        )
        # right_out = cv2.VideoWriter(output_dir + scenario_id + 'right_camera.mp4', fourcc, 10.0, (1920, 1080))
        # Open bag
        bag = rosbag.Bag(bag_path, "r")

        # Iterate through bag
        print("Iterate through bag")
        for topic, msg, t in bag.read_messages():
            # Add time back to the target message header
            if topic == "/front_cam/image_raw":
                pub_front.publish(msg)
            elif topic == "/rear_cam/image_raw":
                pub_rear.publish(msg)
            elif topic == "/left_cam/image_raw":
                pub_left.publish(msg)
            # elif topic == '/right_cam/image_raw':
            # pub_right.publish(msg)

            # Write message to bag
            while flag == True:
                pass
            # print(flag)
            time.sleep(0.0005)

        # Close bag - Very important else you'll have to reindex it
        ts.unregisterCallback()
        print("Close bag")
        bag.close()
        front_out.release()
        rear_out.release()
        left_out.release()
        # right_out.release()


if __name__ == "__main__":

    main()
