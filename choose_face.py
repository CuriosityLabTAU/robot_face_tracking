import rospy
from dynamixel_hr_ros.msg import *
from opencv_apps.msg import FaceArrayStamped
from std_msgs.msg import Float32MultiArray
import numpy as np

class Choose_face():

    def __init__(self):
        rospy.init_node('get_face_list', anonymous=True)
        rospy.Subscriber('/face_detection/faces',FaceArrayStamped , self.callback)
        self.publisher = rospy.Publisher('chosen_face', Float32MultiArray, queue_size=1)
        self.face = Float32MultiArray()

    def callback(self, data):
        msg = data.faces
        if len(msg)<=0:
            return
        self.selection(msg)

    def selection(self, msg):
        x = msg[0].face.x
        y = msg[0].face.y
        self.face.data = [x,y]
        self.publisher.publish(self.face)

    def listener(self):
        rospy.spin()

if __name__ == '__main__':
    choose_face = Choose_face()
    choose_face.listener()
