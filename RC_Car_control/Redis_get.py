#!/usr/bin/env python
#-*- coding:utf-8 -*-

"""
레디스 설치 전
sudo apt-get update
sudo apt-get upgrade
레디스 서버 설치
sudo apt-get install redis-server
레디스 설치 확인
redis-server --version
현재 서버의 총 메모리 확인
vmstat -s
"""
import redis



## Redis 클라이언트 객체 생성
## port와 password 인자를 사용하여 서버 연결 정보 지정
## db는 데이터베이스 번호, 기본값은 0
def get_data():

  try:
    redis_client = redis.StrictRedis(host='j8a408.p.ssafy.io', port=6379, db=0)

    while True:
      # current_velocity_x = redis_client.get('current_velocity')
      # current_brake = redis_client.get('current_brake')
      # destination = redis_client.get('destination')
      # current_acceleration = redis_client.get('current_acceleration')
      # current_position_x = redis_client.get('current_position_x')
      # current_position_y = redis_client.get('current_position_y')
      hi = 1234
      hello = 1515
      return [hi, hello]

    # return [current_velocity_x, current_brake, destination,
    #         current_acceleration, current_position_x, current_position_y]

  except Exception as ex:
    print('Error:' ,ex)
  ## 연결 종료
  ## redis_client.close()


    

