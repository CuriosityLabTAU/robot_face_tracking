import os
import threading
import time
import sys


def start_working():


    def worker_usb_camera():
        print('starting usb camera...')
        os.system('roslaunch usb_cam usb_cam-test.launch')
        return

    def worker_opencv_face_detection():
        print('starting opencv...')
        os.system('roslaunch opencv_apps face_detection.launch')
        return

    def worker_choose_face():
        print('starting choose face...')
        os.system('python choose_face.py')
        return

    def worker_convert_coordinate():
        print('starting convert coordinate using 360...')
        os.system('python convert_coordinates.py %d' % camera_type)

        # print('starting convert coordinate using USB...')
        # os.system('python convert_coordinates.py 1')

        return


    def run_thread(worker):
        threading.Thread(target=worker).start()
        threading._sleep(2.0)

    run_thread(worker_usb_camera)
    run_thread(worker_opencv_face_detection)
    run_thread(worker_choose_face)
    run_thread(worker_convert_coordinate)


if len(sys.argv) > 1:
    camera_type = int(sys.argv[1])
else:
    camera_type = 2

start_working()