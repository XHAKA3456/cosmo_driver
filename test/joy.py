#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import pygame
import time
import math

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoystickTeleop(Node):
    def __init__(self):
        super().__init__('joystick_teleop_node')
        pygame.init()
    
        # 연결된 조이스틱(게임패드) 개수 확인
        joystick_count = pygame.joystick.get_count()
        if joystick_count == 0:
            print("연결된 게임패드가 없습니다.")
            return
        
        # 0번 인덱스 조이스틱 사용(보통 한 개만 연결되어 있다면 0번이 Xbox 패드일 겁니다.)
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        
        # 축/버튼 매핑 (Xbox 컨트롤러 기준 예시)
        # 필요에 따라 실제 축, 버튼에 맞게 인덱스를 변경하세요.
        self.linear_axis_index = 1   # 왼쪽 스틱 상하 (기본적으로 axes[1])
        self.angular_axis_index = 0  # 왼쪽 스틱 좌우 (기본적으로 axes[0])

        # 속도 스케일
        self.linear_scale = 0.5
        self.angular_scale = 2.3

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.publish_vel = False
        self.get_logger().info("JoystickTeleop 노드가 시작되었습니다.")
        self.joy_callback()

    def joy_callback(self):
        
        while True:

            twist = Twist()
            pygame.event.pump()
            # 축(axis) 값 얻기
            btn_A = self.joystick.get_button(0)
            btn_B = self.joystick.get_button(1)
            if btn_A : 
                self.publish_vel = True
            if btn_B:
                self.publish_vel =False
                twist.linear.x = 0.0
                twist.angular.z =0.0
                self.cmd_vel_pub.publish(twist)

            
            linear_x = math.trunc(-self.joystick.get_axis(1) * 10) / 10
            angular_z = math.trunc(self.joystick.get_axis(2) * 10) / 10
            # print ("linear_X : ",linear_x, "  ||||||||||||  ","angular_z : ", angular_z)
            # print("btn _A :"  , btn_A , "  btn _B : " , btn_B)
            if self.publish_vel:
                twist.linear.x = linear_x*self.linear_scale
                twist.angular.z = -angular_z*self.angular_scale
                self.cmd_vel_pub.publish(twist)
                time.sleep(0.01)

def main(args=None):
    rclpy.init(args=args)
    node = JoystickTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

