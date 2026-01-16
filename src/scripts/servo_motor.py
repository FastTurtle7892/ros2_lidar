import time
import board
import busio
from adafruit_servokit import ServoKit

# 1. I2C 버스 초기화 (Jetson Nano의 경우)
i2c = busio.I2C(board.SCL, board.SDA)

# 2. ServoKit 라이브러리 초기화 (채널 16개, 주소 0x60)
kit = ServoKit(channels=16, i2c=i2c)

# ==========================================
# 중요: SG90 서보모터 펄스 폭 설정
# SG90은 보통 500us ~ 2500us 사이의 펄스를 사용합니다.
# 이 설정이 없으면 0도나 180도에 도달하지 못할 수 있습니다.
# ==========================================
kit.servo[0].set_pulse_width_range(500, 2500)

print("SG90 Servo Test Start")

try:
    # 3. 각도 테스트 (0도 -> 90도 -> 180도)
    angles = [0, 90, 180]
    for angle in angles:
        print(f"Moving to {angle} degrees")
        kit.servo[0].angle = angle
        time.sleep(1.0) # 1초 대기

    # 4. 부드럽게 움직이기 (Sweep) 테스트
    print("Sweeping from 0 to 180...")
    for i in range(0, 181, 5): # 5도씩 증가
        kit.servo[0].angle = i
        time.sleep(0.05)
    
    print("Sweeping from 180 to 0...")
    for i in range(180, -1, -5): # 5도씩 감소
        kit.servo[0].angle = i
        time.sleep(0.05)

    # 5. 중앙 정렬 후 종료
    print("Centering servo (90 degrees)")
    kit.servo[0].angle = 90
    time.sleep(0.5)

except KeyboardInterrupt:
    print("\nTest stopped by user")

except Exception as e:
    print(f"\nError: {e}")

finally:
    # 프로그램 종료 시 모터 힘 풀기 (선택 사항)
    # angle을 None으로 설정하면 신호를 끊어 모터가 힘을 뺍니다.
    kit.servo[0].angle = None
    print("Clean up and exit.")
