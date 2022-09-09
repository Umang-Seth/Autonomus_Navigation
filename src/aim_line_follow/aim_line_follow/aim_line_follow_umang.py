import rclpy
from rclpy.node import Node

from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterDescriptor

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
from nxp_cup_interfaces.msg import PixyVector
from time import sleep
from datetime import datetime
import numpy as np
from math import exp

class LineFollow(Node):

    def __init__(self):
        super().__init__('aim_line_follow')



        self.start_delay = 5.0

        self.camera_vector_topic = '/cupcar0/PixyVector'

        self.linear_velocity = 0.8

        self.angular_velocity = -0.68 #-0.75

        self.single_line_steer_scale = 0.4
        #<pose>-.10 0 .2 0 0.8 0</pose>


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
    def speed_scaler(self, x):
        return float((4*exp(-x))/(1+exp(-x))**2)

    def listener_callback(self, msg):
        #TODO
        current_time = datetime.now().timestamp()
        frame_width = 80
        frame_height = 52
        window_center = (frame_width / 2)
        x = 0
        y = 0
        steer = 0
        speed = 0
        num_vectors = self.get_num_vectors(msg)

        if(num_vectors == 0):
            #recovery
            speed, steer = -0.2,0
            
            '''
            if self.restart_time:
                self.start_time = datetime.now().timestamp()
                self.restart_time = False
            if (self.start_time+4.0) > current_time:
                speed = self.linear_velocity * (4.0-(current_time-self.start_time))/4.0
            if (self.start_time+4.0) <= current_time:
                speed = 0
            steer = 0
            '''

        if(num_vectors == 1):
            x = (msg.m0_x1 - msg.m0_x0) / frame_width
            y = (msg.m0_y1 - msg.m0_y0) / frame_height
            if(msg.m0_x0 != msg.m0_x1 and y != 0):
                steer = (-self.angular_velocity)*(x/y)*self.single_line_steer_scale
                speed = self.linear_velocity
                #speed = 0.19
            else:
                #recovery
                steer, speed = 0,0

        if(num_vectors == 2):
            m_x1 = 1.1*(msg.m0_x1 + msg.m1_x1 + msg.m0_x0 + msg.m1_x0) / 4
            steer = self.angular_velocity*(m_x1 - window_center)/window_center
            speed = self.linear_velocity
            self.get_logger().info('m_x1 = '+ str(m_x1))

        scaled_speed = 0.18 + speed*self.speed_scaler(steer)
        self.speed_vector.x = scaled_speed
        self.steer_vector.z = float(steer)
        #self.get_logger().info('speed='+ str(scaled_speed)+', steer='+str(steer))
        

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