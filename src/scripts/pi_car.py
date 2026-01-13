#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import RPi.GPIO as GPIO
import math
import time

# ==========================================
# [선생님의 핀 설정 (BOARD 모드)]
# ==========================================
Motor_A_EN = 7
Motor_B_EN = 11
Motor_A_Pin1 = 37
Motor_A_Pin2 = 40
Motor_B_Pin1 = 12
Motor_B_Pin2 = 13
# ==========================================

class PiCarDriver(Node):
    def __init__(self):
        super().__init__('picar_driver')

        # 1. GPIO 초기화
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD) # 물리 핀 번호 사용

        GPIO.setup(Motor_A_EN, GPIO.OUT)
        GPIO.setup(Motor_B_EN, GPIO.OUT)
        GPIO.setup(Motor_A_Pin1, GPIO.OUT)
        GPIO.setup(Motor_A_Pin2, GPIO.OUT)
        GPIO.setup(Motor_B_Pin1, GPIO.OUT)
        GPIO.setup(Motor_B_Pin2, GPIO.OUT)

        self.pwm_A = GPIO.PWM(Motor_A_EN, 1000)
        self.pwm_B = GPIO.PWM(Motor_B_EN, 1000)
        self.pwm_A.start(0)
        self.pwm_B.start(0)
        self.LINEAR_SCALE = 0.75
        self.ANGULAR_SCALE = 0.75
        
        # 2. ROS 통신 설정
        self.create_subscription(Twist, '/cmd_vel', self.listener_callback, 10)
        
        # [수정] Lidar Odometry가 Odom을 대신 발행하므로, 여기서는 충돌 방지를 위해 비활성화합니다.
        # self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        
        # [수정] TF도 Lidar Odometry가 담당하므로 비활성화합니다.
        # self.tf_broadcaster = TransformBroadcaster(self)

        # 위치 계산용 변수 (로직은 남겨두지만 방송은 하지 않음)
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()
        self.current_linear_x = 0.0
        self.current_angular_z = 0.0

        # 0.02초마다 위치 업데이트 (기존 로직 유지)
        self.create_timer(0.02, self.update_odometry)
        self.get_logger().info(">>> PiCar Driver (Board Pin) Ready! - Wheel Odom Disabled")

    def listener_callback(self, msg):
        self.current_linear_x = msg.linear.x
        self.current_angular_z = msg.angular.z

        # 믹싱 알고리즘
        left_speed = (msg.linear.x + msg.angular.z) * 100
        right_speed = (msg.linear.x - msg.angular.z) * 100

        self.control_motor('A', left_speed)
        self.control_motor('B', right_speed)

    def control_motor(self, motor_name, speed):
        speed = max(min(speed, 100), -100)
        abs_speed = abs(speed)

        # A모터 (왼쪽)
        if motor_name == 'A':
            if speed > 0:
                GPIO.output(Motor_A_Pin1, GPIO.HIGH)
                GPIO.output(Motor_A_Pin2, GPIO.LOW)
            elif speed < 0:
                GPIO.output(Motor_A_Pin1, GPIO.LOW)
                GPIO.output(Motor_A_Pin2, GPIO.HIGH)
            else:
                GPIO.output(Motor_A_Pin1, GPIO.LOW)
                GPIO.output(Motor_A_Pin2, GPIO.LOW)
            self.pwm_A.ChangeDutyCycle(abs_speed)

        # B모터 (오른쪽)
        elif motor_name == 'B':
            if speed > 0: # 선생님 코드대로 (LOW, HIGH가 전진)
                GPIO.output(Motor_B_Pin1, GPIO.LOW)
                GPIO.output(Motor_B_Pin2, GPIO.HIGH)
            elif speed < 0:
                GPIO.output(Motor_B_Pin1, GPIO.HIGH)
                GPIO.output(Motor_B_Pin2, GPIO.LOW)
            else:
                GPIO.output(Motor_B_Pin1, GPIO.LOW)
                GPIO.output(Motor_B_Pin2, GPIO.LOW)
            self.pwm_B.ChangeDutyCycle(abs_speed)

    def update_odometry(self):
        # 이동량 계산 (명령어 기반 추측)
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if self.current_linear_x == 0 and self.current_angular_z == 0:
            delta_x = 0
            delta_y = 0
            delta_th = 0
        else:
            # 보정 계수를 곱해서 계산
            adjusted_linear = self.current_linear_x * self.LINEAR_SCALE
            adjusted_angular = self.current_angular_z * self.ANGULAR_SCALE

            delta_x = (adjusted_linear * math.cos(self.th)) * dt
            delta_y = (adjusted_linear * math.sin(self.th)) * dt
            delta_th = adjusted_angular * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        # [중요 수정] TF 방송 코드 주석 처리
        # Lidar Odometry가 정확한 위치를 계산해 TF를 보내므로,
        # 여기서 보내면 두 정보가 충돌하여 로봇이 떨리게 됩니다.
        """
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(self.th / 2.0)
        t.transform.rotation.w = math.cos(self.th / 2.0)
        self.tf_broadcaster.sendTransform(t)

        # Odom 토픽
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = t.transform.rotation
        self.odom_publisher.publish(odom)
        """

    def stop(self):
        self.pwm_A.ChangeDutyCycle(0)
        self.pwm_B.ChangeDutyCycle(0)
        GPIO.cleanup()

def main(args=None):
    rclpy.init(args=args)
    picar = PiCarDriver()
    try:
        rclpy.spin(picar)
    except KeyboardInterrupt:
        picar.stop()
        picar.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
