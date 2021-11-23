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
        self.old_face = [100, 100]

    def callback(self, data):
        msg = data.faces
        if len(msg)<=0:
            return
        self.selection(msg)

    def selection(self, msg):
        if len(msg)>1:
            p_0 = [msg[0].face.x, msg[0].face.y]
            p_1 = [msg[1].face.x, msg[1].face.y]
            d_0 = np.linalg.norm(self.old_face, p_0)
            d_1 = np.linalg.norm(self.old_face, p_1)
            if d_0 > d_1:
                x = p_1[0]
                y = p_1[1]
            else:
                x = p_0[0]
                y = p_0[1]
        else:
            x = msg[0].face.x
            y = msg[0].face.y
        self.old_face = [x,y]
        self.face.data = [x,y]
        self.publisher.publish(self.face)

    def listener(self):
        rospy.spin()

if __name__ == '__main__':
    choose_face = Choose_face()
    choose_face.listener()
