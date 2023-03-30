import time
import threading
import os
import sys
import socket
import RPi.GPIO as GPIO
from Door_ServoMotor import openclose
from DC_motor import DC_MOTOR
from servo_motor import SERVO_MOTOR
import redis


class main():
    def __init__(self):
        pass

    redis_client = redis.StrictRedis(host='j8a408.p.ssafy.io', port=6379, db=0, password='carming123')

    ## redis에서 데이터 확인
    while True:
        ## 속도 b'1.4650933742523193' 형태로 출력
        current_velocity = redis_client.get('current_velocity')


        ## 정지 b'1.0' 형태로 출력
        current_brake = redis_client.get('current_brake')
        ## destination = redis_client.get('destination')
        current_acceleration = redis_client.get('current_acceleration')
        current_position_x = redis_client.get('current_position_x')
        current_position_y = redis_client.get('current_position_y')

        ## 모터의 출력율로 변환 NT761XDA-X16/C(대충 20이하의 값이 나오므로 5배 처리)
        ## drive 함수에 해당 값을 인수로 넣어서 하면 된다.
        speed = 5 * current_velocity
        DC_MOTOR.drive(speed)
        ## print(current_velocity)
        ### 계속해서 함수를 작동시키는 대신 , sleep을 걸어줘야함 / 또는 이 while true문 자체를 sleep

        ## 조향 자체가 어떻게 찍히는 지 확인해서 dutycycle 변수 입력
        SERVO_MOTOR.steering(dutycycle)

        ## 시뮬레이터상에터 brake 동작한 경우 문열림, 문닫힘 구현
        if current_brake == b'1.0':
            print('okok')
            openclose()


if __name__ == "__main__":
    ## Global Variable 선언
    global input_1, input_2, enable, servo_pin
    input_1 = 15
    input_2 = 18
    enable = 27
    servo_pin = 17
    current_state = []

    main()

