#!/usr/bin/env python

import rospy
from morai_msgs.msg import EgoVehicleStatus
from morai_msgs.srv import MoraiEventCmdSrv
import redis
import os

redis_client = redis.Redis(host='j8a408.p.ssafy.io', port=6379, db=0, password='carming123')

class Realtime_listener:
    def __init__(self):
        rospy.init_node("is_change_destination", anonymous=True)

        # subscribe to a Redis channel
        pubsub = redis_client.pubsub()
        pubsub.subscribe('destination_coordinate')

        self.data = {}
        rate = rospy.Rate(2) # 2 times / 1 sec

        while not rospy.is_shutdown():
            for message in pubsub.listen():
                # process the message when there is a new message
                if message['type'] == 'message':
                    self.data = eval(message['data']) # convert string to dictionary
                    print("-----------------------")
                    # execute roslaunch command
                    os.system("roslaunch ssafy_2 Adaptive_Cruise_Control.launch")
            rate.sleep()



if __name__ == "__main__":
    try:
        start = Realtime_listener()

    except rospy.ROSInterruptException:
        pass