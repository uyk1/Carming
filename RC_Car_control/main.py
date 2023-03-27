import time
import threading
import os
import sys
import socket
from Door_ServoMotor import openclose
import Redis_get


class main():
    def __init__(self):
        pass

    ## 계속 실행시켜야함
    ## [속도,정지,목적지도달,가속도,현재위치x,현재위치y]

    current_state = Redis_get.get_data()
    if current_state[1] == 1:
        print(current_state[1])
        openclose()


if __name__ == "__main__":
    ## Global Variable 선언
    global input_1, input_2, enable, servo_pin
    global steering, drive  ## 각각 조향/주행 함수
    input_1 = 15
    input_2 = 18
    enable = 27
    servo_pin = 17
    current_state = []
    main()

