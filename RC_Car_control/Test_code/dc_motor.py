import RPi.GPIO as GPIO
import time

# DC 모터 핀 설정
DC_pin1 = 15
DC_pin2 = 18
enable = 27

# GPIO 초기화
GPIO.setmode(GPIO.BCM)
GPIO.setup(DC_pin1, GPIO.OUT)
GPIO.setup(DC_pin2, GPIO.OUT)
GPIO.setup(enable, GPIO.OUT)

# DC 모터 제어 함수
try:
    pwm = GPIO.PWM(enable, 100)
    print('1')
    pwm.start(0)
    print('2')
    while True:
        # speed 값에 따라 DC 모터 속도 조절
        pwm.ChangeDutyCycle(50)
        print('3')
        GPIO.output(DC_pin1, GPIO.HIGH)
        print('4')
        GPIO.output(DC_pin2, GPIO.LOW)
        print('5')
        time.sleep(2)
        print('6')
except KeyboardInterrupt:
    GPIO.cleanup()
    print('7')
