from datetime import datetime
from time import sleep

import numpy as np
import rclpy
from geometry_msgs.msg import Twist, Vector3
from nxp_cup_interfaces.msg import PixyVector
from rcl_interfaces.msg import Parameter, ParameterDescriptor, ParameterType
from rclpy.exceptions import ParameterNotDeclaredException
from rclpy.node import Node
from std_msgs.msg import Float64


class LineFollow(Node):

    def __init__(self):
        super().__init__('aim_line_follow')


        self.start_delay =5.0

        self.camera_vector_topic ="/cupcar0/PixyVector"

        self.linear_velocity = 1.1

        self.angular_velocity = -3.8

        self.single_line_steer_scale = 0.45

        

        # Time to wait before running
        self.get_logger().info('Waiting to start for {:s}'.format(str(self.start_delay)))
        sleep(self.start_delay)
        self.get_logger().info('Started')

        self.start_time = datetime.now().timestamp()
        self.restart_time = True

        # Subscribers
        self.pixy_subscriber = self.create_subscription(
            PixyVector,
            self.camera_vector_topic,
            self.listener_callback,
            10)

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cupcar0/cmd_vel', 10)

        self.speed_vector = Vector3()
        self.steer_vector = Vector3()
        self.cmd_vel = Twist()

        # Timer setup
        # timer_period = 0.5 #seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.i = 0

    def get_num_vectors(self, msg):
        num_vectors = 0
        if(not(msg.m0_x0 == 0 and msg.m0_x1 == 0 and msg.m0_y0 == 0 and msg.m0_y1 == 0)):
            num_vectors = num_vectors + 1
        if(not(msg.m1_x0 == 0 and msg.m1_x1 == 0 and msg.m1_y0 == 0 and msg.m1_y1 == 0)):
            num_vectors = num_vectors + 1
        return num_vectors

    # def timer_callback(self):
    #     #TODO

    def listener_callback(self, msg):
        #TODO
        current_time = datetime.now().timestamp()
        frame_width =79
        frame_height = 52
        window_center = (frame_width / 2)
        x = 0
        y = 0
        steer = 0
        speed = 0
        num_vectors = self.get_num_vectors(msg)

        if(num_vectors == 0):
            """
            if self.restart_time:
                self.start_time = datetime.now().timestamp()
                self.restart_time = False
            if (self.start_time+2.0) > current_time:
                speed = self.linear_velocity * (4.0-(current_time-self.start_time))/4.0
            if (self.start_time+2.0) <= current_time:
            """
            speed = 0
            steer = 0

        if(num_vectors == 1):
            if(msg.m0_x1 > 40 or 0.25 >= msg.m0_y0 > 0.0):
                steer = 0.5*self.single_line_steer_scale #ang   #Right 
                speed = 0.3 #linear
            else:
                steer = -0.5*self.single_line_steer_scale #ang # Left
                speed = 0.3  #linear

            # if(msg.m0_x0 < 40 or 0.3 < msg.m0_y1 < 0.1):
            #     steer = -0.3 #ang   #Right 
            #     speed = 0.5 #linear
            # if(msg.m1_x0 < 40 or 0.3 < msg.m1_y1 < 0.1):
            #     steer = 0.3 #ang # Left
            #     speed = 0.5 #linear
            # else:
            #     steer = 0.3
            #     speed = 0.5

        #     if not self.restart_time:
        #         self.start_time = datetime.now().timestamp()
        #         self.restart_time = True
        #     if(msg.m0_x1 > msg.m0_x0):
        #         x = (msg.m0_x1 - msg.m0_x0) / frame_width
        #         y = (msg.m0_y1 - msg.m0_y0) / frame_height
        #     else:
        #         x = (msg.m0_x0 - msg.m0_x1) / frame_width
        #         y = (msg.m0_y0 - msg.m0_y1) / frame_height
        #     if(msg.m0_x0 != msg.m0_x1 and y != 0):
        #         steer = (-self.angular_velocity) * (x / y) * self.single_line_steer_scale
        #         if (self.start_time+4.0) > current_time:
        #             speed = self.linear_velocity * ((current_time-self.start_time)/4.0)
        #         if (self.start_time+4.0) <= current_time:
        #             speed = self.linear_velocity
        #     else:
        #         steer = 0
        #         if (self.start_time+4.0) > current_time:
        #             speed = self.linear_velocity * ((current_time-self.start_time)/4.0)*0.9
        #         if (self.start_time+4.0) <= current_time:
        #             speed = self.linear_velocity*0.9

        if(num_vectors == 2):
            if not self.restart_time:
                self.start_time = datetime.now().timestamp()
                self.restart_time = True
            # m_x1 = (msg.m0_x1 + msg.m1_x1) / 2

            m_x1 = 1.1*(msg.m0_x1 + msg.m1_x1 + msg.m0_x0 + msg.m1_x0) / 4

            steer = self.angular_velocity*(m_x1 - window_center) / frame_width
            if (self.start_time+4.0) > current_time:
                speed = self.linear_velocity * ((current_time-self.start_time)/4.0)
            if (self.start_time+4.0) <= current_time:
                speed = self.linear_velocity

        self.speed_vector.x = float(speed*(1-np.abs(1.45*steer)))
        self.steer_vector.z = float(steer)

        self.cmd_vel.linear = self.speed_vector
        self.cmd_vel.angular = self.steer_vector

        self.cmd_vel_publisher.publish(self.cmd_vel)

def main(args=None):
    rclpy.init(args=args)

    line_follow = LineFollow()

    rclpy.spin(line_follow)

    line_follow.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

"""
# Vector 0 head and tail points
msg.m0_x0    # Tail of vector @ x cordinate
msg.m0_y0    # Tail of vector @ y cordinate
msg.m0_x1    # Head of vector @ x cordinate
msg.m0_y1    # Head of vector @ y cordinate

# Vector 1 head and tail points
msg.m1_x0    # Tail of vector @ x cordinate
msg.m1_y0    # Tail of vector @ y cordinate
msg.m1_x1    # Head of vector @ x cordinate
msg.m1_y1    # Head of vector @ y cordinate
"""