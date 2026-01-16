#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_servokit import ServoKit
import time

# ==========================================
# [모터 드라이버 설정 클래스]
# keyboard_motor.py 및 motor_test.py 참고
# ==========================================
class PWMThrottleHat:
    def __init__(self, pwm, channel):
        self.pwm = pwm
        self.channel = channel
        self.pwm.frequency = 60

    def set_throttle(self, throttle):
        # 입력 범위: -1.0 ~ 1.0
        # motor_test.py에 따르면:
        # throttle < 0 : 전진 (Forward)
        # throttle > 0 : 후진 (Backward)
        
        throttle = max(min(throttle, 1.0), -1.0) # 범위 제한
        pulse = int(0xFFFF * abs(throttle))

        if throttle < 0: # 전진
            self.pwm.channels[self.channel + 5].duty_cycle = pulse
            self.pwm.channels[self.channel + 4].duty_cycle = 0
            self.pwm.channels[self.channel + 3].duty_cycle = 0xFFFF
        elif throttle > 0: # 후진
            self.pwm.channels[self.channel + 5].duty_cycle = pulse
            self.pwm.channels[self.channel + 4].duty_cycle = 0xFFFF
            self.pwm.channels[self.channel + 3].duty_cycle = 0
        else: # 정지
            self.pwm.channels[self.channel + 5].duty_cycle = 0
            self.pwm.channels[self.channel + 4].duty_cycle = 0
            self.pwm.channels[self.channel + 3].duty_cycle = 0

class JetsonCarDriver(Node):
    def __init__(self):
        super().__init__('jetson_car_driver')

        # 1. I2C 및 하드웨어 초기화
        try:
            self.i2c = busio.I2C(board.SCL, board.SDA)
            
            # DC 모터 설정을 위한 PCA9685
            self.pca = PCA9685(self.i2c)
            self.pca.frequency = 60
            self.motor_hat = PWMThrottleHat(self.pca, channel=0)
            
            # 서보 모터 설정을 위한 ServoKit
            self.kit = ServoKit(channels=16, i2c=self.i2c)
            # servo_motor.py 파일에 명시된 펄스 폭 설정 적용
            self.kit.servo[0].set_pulse_width_range(500, 2500)
            
            self.get_logger().info(">>> Hardware Initialized: PCA9685 & ServoKit")
        except Exception as e:
            self.get_logger().error(f"Hardware Initialization Failed: {e}")
            return

        # 2. 파라미터 설정
        self.MAX_THROTTLE = 0.7  # 최대 속도 제한 (0.0 ~ 1.0)
        self.STEERING_CENTER = 100 # keyboard_motor.py 초기값 기준 (필요시 90으로 조정)
        self.MAX_STEERING_RANGE = 40 # 중앙에서 좌우로 움직일 최대 각도
        
        # 3. ROS 통신 설정
        self.create_subscription(Twist, '/cmd_vel', self.listener_callback, 10)
        
        # [참고] Odometry 및 TF는 Lidar 패키지(rf2o)에서 처리하므로 여기서는 모터 제어만 수행합니다.

        self.current_throttle = 0.0
        self.current_steering_angle = self.STEERING_CENTER
        
        # 초기 서보 정렬
        self.kit.servo[0].angle = self.STEERING_CENTER
        self.get_logger().info(">>> Jetson Orin Car Driver Ready! (Ackermann Steering)")

    def listener_callback(self, msg):
        # 1. 속도 제어 (DC 모터)
        # ROS: linear.x > 0 (전진), linear.x < 0 (후진)
        # 하드웨어: throttle < 0 (전진), throttle > 0 (후진) -> 부호 반전 필요
        linear_vel = msg.linear.x
        
        # 간단한 비례 제어 (속도값이 너무 크면 MAX_THROTTLE로 제한)
        # linear.x가 1.0일 때 MAX_THROTTLE이 되도록 설정하거나, 값을 그대로 매핑
        target_throttle = -linear_vel  # 부호 반전 (하드웨어 특성 반영)
        
        # 안전을 위해 클램핑
        if target_throttle > self.MAX_THROTTLE: target_throttle = self.MAX_THROTTLE
        if target_throttle < -self.MAX_THROTTLE: target_throttle = -self.MAX_THROTTLE
        
        self.motor_hat.set_throttle(target_throttle)

        # 2. 조향 제어 (서보 모터)
        # ROS: angular.z > 0 (좌회전/반시계), angular.z < 0 (우회전/시계)
        # keyboard_motor.py 분석:
        # 'a' (Left) -> pan 감소 (-= 10)
        # 'd' (Right) -> pan 증가 (+= 10)
        # 따라서: angular.z 양수(좌회전) -> 각도 감소, angular.z 음수(우회전) -> 각도 증가
        
        angular_vel = msg.angular.z
        
        # 각도 변환 (회전 속도 rad/s를 각도 오프셋으로 변환)
        # 계수 30.0은 민감도 조절용 (테스트하며 튜닝 필요)
        steering_offset = angular_vel * 40.0 
        
        # 좌회전(양수 입력)일 때 각도를 줄여야 하므로 뺄셈
        target_angle = self.STEERING_CENTER - steering_offset
        
        # 서보 물리적 한계 제한 (0~180도, 또는 설정한 범위 내)
        min_angle = self.STEERING_CENTER - self.MAX_STEERING_RANGE
        max_angle = self.STEERING_CENTER + self.MAX_STEERING_RANGE
        
        target_angle = max(min(target_angle, max_angle), min_angle)
        
        # 값 적용
        self.kit.servo[0].angle = target_angle

        # 디버깅용 로그 (필요시 주석 해제)
        # self.get_logger().info(f"V: {linear_vel:.2f} -> T: {target_throttle:.2f} | W: {angular_vel:.2f} -> Ang: {target_angle:.2f}")

    def stop(self):
        # 종료 시 모터 정지 및 서보 중앙 정렬
        self.motor_hat.set_throttle(0)
        self.kit.servo[0].angle = self.STEERING_CENTER
        self.pca.deinit()

def main(args=None):
    rclpy.init(args=args)
    car = JetsonCarDriver()
    try:
        rclpy.spin(car)
    except KeyboardInterrupt:
        pass
    finally:
        car.stop()
        car.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
