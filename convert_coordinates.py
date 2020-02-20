import rospy
from dynamixel_hr_ros.msg import *
from opencv_apps.msg import FaceArrayStamped
from std_msgs.msg import Float32MultiArray
import numpy as np
import sys

camera_type = 1  # 1 = standard usb camera; 2 = 360 camera behind the robot on th left side; 3- 360 camera infront of the robot

class Convert_coordinates():

    def __init__(self):
        rospy.init_node('convert_coordinates', anonymous=True)
        rospy.Subscriber('chosen_face', Float32MultiArray, self.callback)
        # self.publisher = rospy.Publisher('gaze_angles', Float32MultiArray, queue_size=1)
        # self.face = Float32MultiArray()
        self.publisher = rospy.Publisher('/patricc_face_tracking', CommandPosition, queue_size=10)
        self.motor_list = {'skeleton': [0, 1, 4, 5, 6, 7], 'head_pose': [2], 'lip': [3],
                           'full': [0, 1, 2, 3, 4, 5, 6, 7], 'full_idx': [1, 2, 3, 4, 5, 6, 7, 8]}
        self.robot_angle_range = [[0.0, 5.0],  # [1.1, 3.9],
                                  [2.8, 1.6],
                                  [2, 3.3], [2.2, 2.5],  # [1.8, 2.5], #[2.5, 3.5], #
                                  [4.1, 0.9], [1.3, 3],
                                  [1, 4.1], [2.5, 3.75]]
        self.middle = [(self.robot_angle_range[i][0] + self.robot_angle_range[i][1]) / 2.0 for i in range(8)]
        self.l = 0.4  # distance from user to robot

        self.head_body = 2  # 1 = control gaze through body pitch and yaw; 2 = control gaze through neck motor
        if camera_type == 1:  # for standard usb camera
            self.xCal = 325
            self.yCal = 130
            self.video_range = [[130 - self.xCal, 525 - self.xCal], [130 - self.yCal, 370 - self.yCal]]
        if camera_type == 2:  # for 360 camera
            self.xCal = 325
            self.yCal = 270
            self.video_range = [[1060, 920], [330, 210]]

        self.real_range = [[-0.5, 0.5], [0.0, 1.0]]
        self.angle_range = [[-np.pi / 4, np.pi / 4], [0, np.pi / 4]]
        self.robot_x_range = [3.1, 2]  # [3.5, 1.7]
        self.robot_y_range = [2.2, 2.9]
        self.robot_neck_range = [2.2, 3.1]
        print 'camera type = ', camera_type
        if camera_type == 1:  # for standard usb camera
            self.video_x_range = [525, 130]
            self.video_y_range = [370, 130]
        if camera_type == 2:  # for 360 camera
            self.video_x_range = [1060, 920]
            self.video_y_range = [330, 210]
        if camera_type == 3:
            self.video_x_range = [400, 150]
            self.video_y_range = [32, 62]


    def map_angles(self, video_range, real_range, pos_video):
        pos_real = real_range[0] + (pos_video - video_range[0]) * (
                    (real_range[1] - real_range[0]) / (video_range[1] - video_range[0]))
        return pos_real

    def callback(self, data):
        msg = data.data
        # if len(msg)<=0:
        #    return
        x = msg[0]
        y = msg[1]

        # yaw, pitch = self.xy2angles(x,y)
        yaw_robot = self.map_angles(self.video_x_range, self.robot_x_range, x)
        pitch_robot = self.map_angles(self.video_y_range, self.robot_y_range, y)
        neck_robot = self.map_angles(self.video_x_range, self.robot_neck_range, x)
        print x, y
        #print "pixel", x, ",", y, "angles", yaw_robot, ",", pitch_robot, ",", neck_robot
        self.sendCommand(yaw_robot, pitch_robot, neck_robot, self.head_body)

    def sendCommand(self, yaw_robot, pitch_robot, neck_robot, head_body):
        new_command = CommandPosition()
        new_command.id = [i for i in range(1, 9)]
        new_command.angle = [-999 for i in range(1, 9)]
        if head_body == 1:  # controls gaze through yaw and pitch base motors
            # new_command.angle = [yaw_robot, pitch_robot] + self.middle[2:8]
            new_command.angle[0:1] = [yaw_robot, pitch_robot]
        if head_body == 2:  # controls gaze through neck motor
            # temp_angle = self.middle
            # temp_angle[0:3] = [2.6, 2.7, neck_robot]
            # new_command.angle = temp_angle
            new_command.angle[2] = neck_robot

        #print "####", new_command.angle
        new_command.speed = [1.5, 1.5, 2, 7, 5, 5, 5, 5]
        #print "yaw:", yaw_robot, "pitch:", pitch_robot, "neck angle", neck_robot
        self.publisher.publish(new_command)

    def xy2angles(self, xRaw, yRaw):
        x = xRaw - self.xCal
        y = yRaw - self.yCal

        xReal = self.map_angles(self.video_range[0], self.real_range[0], x)
        yReal = self.map_angles(self.video_range[1], self.real_range[1], y)

        yaw = np.arctan2(xReal, self.l)
        s = np.sqrt(np.power(xReal, 2) + np.power(yReal, 2))
        pitch = np.arctan2(yReal, s)
        return yaw, pitch

    def listener(self):
        rospy.spin()


if __name__ == '__main__':
    if len(sys.argv) > 1:
        camera_type = int(sys.argv[1])
    convert_coordinates = Convert_coordinates()
    convert_coordinates.listener()
