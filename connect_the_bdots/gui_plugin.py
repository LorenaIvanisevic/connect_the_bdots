import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import math


class RobotArm(Node):
    def __init__(self):
        super().__init__('my_python_node')
        self.declare_parameter('image_file', 'roboticArm.png')
        file_loc = self.get_parameter('image_file').get_parameter_value().string_value
        self.image = cv2.imread(file_loc)
        self.kin = 1 # 1=direct 0=inverse
        self.image = cv2.resize(self.image, (0, 0), fx=0.9, fy=0.9)
        self.timer = self.create_timer(0.1, self.timer_callback)
        cv2.namedWindow('Robotic Arm', cv2.WINDOW_NORMAL | cv2.WINDOW_GUI_EXPANDED)
        
        cv2.createButton('Direct kinematics', self.on_check_d, None, cv2.QT_RADIOBOX, 1)
        cv2.createButton('Inverse kinematics', self.on_check_i, None, cv2.QT_RADIOBOX, 0)
        cv2.createButton('Start position', self.startPosition, None, cv2.QT_PUSH_BUTTON,1)
        cv2.createButton('Autonomous mode', self.autoMode, None, cv2.QT_PUSH_BUTTON,1)

        cv2.createTrackbar('q1','Robotic Arm',0,135,self.slider_change)
        cv2.setTrackbarMin('q1', 'Robotic Arm', -135)
        cv2.createTrackbar('q2','Robotic Arm',0,65,self.slider_change)
        cv2.setTrackbarMin('q2', 'Robotic Arm', -65)
        cv2.createTrackbar('q3','Robotic Arm',0,150,self.slider_change)
        cv2.setTrackbarMin('q3', 'Robotic Arm', -150)
        cv2.createTrackbar('x','Robotic Arm',0,175,self.slider_change)
        cv2.setTrackbarMin('x', 'Robotic Arm', -175)
        cv2.createTrackbar('y','Robotic Arm',0,275,self.slider_change)
        cv2.setTrackbarMin('y', 'Robotic Arm', -75)
        cv2.createTrackbar('z','Robotic Arm',0,350,self.slider_change)

        cv2.imshow('Robotic Arm', self.image)
        self.text = "Default algorithm is direct kinematics, to change it pres Ctr+P"
        cv2.setTrackbarPos('x','Robotic Arm', 0)
        cv2.setTrackbarPos('y','Robotic Arm', 76)
        cv2.setTrackbarPos('z','Robotic Arm', 171)
        cv2.setTrackbarPos('q1','Robotic Arm', 0)
        cv2.setTrackbarPos('q2','Robotic Arm', -60)
        cv2.setTrackbarPos('q3','Robotic Arm', 150)


        self.block_sliders()


    def on_check_d(self,state, userdata=None):
        self.kin = 1
        print("direct kinematics")
        self.block_sliders()


    def on_check_i(self,state, userdata=None):
        self.kin = 0
        print("inverse kinematic")   
        self.block_sliders() 


    def block_sliders(self):
        if (self.kin):
            cv2.setTrackbarPos('x','Robotic Arm', 0)
            cv2.setTrackbarPos('y','Robotic Arm', 76)
            cv2.setTrackbarPos('z','Robotic Arm', 171)   
        else:
            cv2.setTrackbarPos('q1','Robotic Arm', 0)
            cv2.setTrackbarPos('q2','Robotic Arm', -60)
            cv2.setTrackbarPos('q3','Robotic Arm', 150)


    def slider_change(self, pos=None):
        pass
        

    def autoMode(self,state, userdata=None):
        p = 10
        connect_arr = [(100,300,10,10), (300,100,20,20)]
        avoid_arr = [(200,200,20+p,20+p)]
        img = np.ones((350,350))
        for object in connect_arr:
            img[object[0]:object[0]+object[2],object[1]:object[1]+object[3]] = 0
        for object in avoid_arr:
            img[object[0]:object[0]+object[2],object[1]:object[1]+object[3]] = 0
        
        d = 150
        x = 350-d
        y = int((350-d)/2)
        img[x:x+d, y:y+d] = 0
        return


    def startPosition (self, state, userdata=None):
        cv2.setTrackbarPos('x','Robotic Arm', 0)
        cv2.setTrackbarPos('y','Robotic Arm', 76)
        cv2.setTrackbarPos('z','Robotic Arm', 171)
        cv2.setTrackbarPos('q1','Robotic Arm', 0)
        cv2.setTrackbarPos('q2','Robotic Arm', -60)
        cv2.setTrackbarPos('q3','Robotic Arm', 150)


    def timer_callback(self):

        display_image = self.image.copy()
        imh, imw= display_image.shape[:2]
        extended_image = np.full(
            (imh + 300, imw + 300, 3),
            (240,240,240),
            dtype=np.uint8
        )
        extended_image[0:imh, 150:150 + imw] = display_image
        overlay = extended_image.copy()
        cv2.rectangle(overlay, (0, extended_image.shape[0]-200), 
                    (extended_image.shape[1], extended_image.shape[0]), 
                    (240,240,240), -1)
        cv2.addWeighted(overlay, 1, extended_image, 0, 0, extended_image)
        text_position = (150, extended_image.shape[0] - 100) 
        cv2.putText(extended_image, self.text, text_position, cv2.FONT_HERSHEY_SIMPLEX, 
                    0.7, (0,0,0), 2, cv2.LINE_AA)

        cv2.imshow('Robotic Arm', extended_image)

        if self.kin:
            q1 = cv2.getTrackbarPos('q1','Robotic Arm')
            q2 = cv2.getTrackbarPos('q2','Robotic Arm')
            q3 = cv2.getTrackbarPos('q3','Robotic Arm')
            x,y,z = direct_kinematics(q1,q2,q3)
            self.text = "Direct kinematics for (q1 = "+ str(q1) + ", q2 = " + str(q2) + ", q3 = " + str(q3) + ") : (x,y,z) = (" + str(x) + "," + str(y) + "," + str(z) + ")" 
        else:
            x = cv2.getTrackbarPos('x','Robotic Arm')
            y = cv2.getTrackbarPos('y','Robotic Arm')
            z = cv2.getTrackbarPos('z','Robotic Arm')
            q1,q2,q3 = inverse_kinematics(x,y,z)
            self.text = "Inverse kinematics for (x = "+ str(x) + ", y = " + str(y) + ", z = " + str(z) + ") : (q1,q2,q3) = (" + str(q1) + "," + str(q2) + "," + str(q3) + ")" 

        self.block_sliders()
        cv2.waitKey(1)  

def direct_kinematics (q1,q2,q3):
    a1 = 82
    a2 = 178
    a3 = 230
    q1 = math.radians(q1)
    q2 = math.radians(q2)
    q3 = math.radians(q3)
    
    x = -math.sin(q1) * (a3*math.sin(q2+q3)+ a2*math.sin(q2))
    y = math.cos(q1) * (a3*math.sin(q2+q3) + a2*math.sin(q2))
    z = a1 + a3*math.cos(q2+q3) + a2*math.cos(q2)
    return round(x,2),round(y,2),round(z,2)


def inverse_kinematics (x,y,z):
    a1 = 82
    a2 = 178
    a3 = 230
    print(x,y,z)
    try:
        q1 = 57.2958 * math.atan2(x,y)
        q2 = 90.0000-57.2958*math.acos((0.5000*(a2**2 - a3**2 + x**2 + y**2 + (a1-z)**2))/(a2*(x**2+y**2+(a1-z)**2)**0.5000)) - 57.2958*math.atan2(1*z-1*a1,(x**2+y**2)**(1/2))
        q3 = 57.2958*math.acos((0.5000*(x**2-a3**2 - a2**2 + y**2 + (a1-z)**2))/(a2*a3))
    except ValueError:
        print("position not posible")
        return "err","err","err"
    return q1,q2,q3


def main(args=None):
    rclpy.init(args=args)
    robot_arm = RobotArm()
    rclpy.spin(robot_arm)
    robot_arm.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
