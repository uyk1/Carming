import time
import threading
import os
import sys
import socket
from DC_motor import DC_MOTOR
from servo_motor import SERVO_MOTOR
from Door_ServoMotor import DOOR
class main():
    def __init__(self):
        pass




if __name__ == "__main__":
    ## Global Variable 선언
    global input_1, input_2, enable, servo_pin
    global steering, drive  ## 각각 조향/주행 함수
    
    input_1 = 15
    input_2 = 18
    enable = 27
    servo_pin = 17
    
    pass
