#!/usr/bin/env python
import math
import os
import os.path as path
import rospy
import threading
import numpy as np
import pandas as pd
from datetime import datetime, timedelta
from geometry_msgs.msg import Twist, Vector3

REGULATOR_POSITION_TOPIC = "/AUVInternalSystem/Position/globalEstimatedPosition"
SET_POSITION_TOPIC = "/AUVInternalSystem/DevPC/arbitrarlySetGlobalPosition"
OBSERVATION_TIME = 60

COLUMNS = ["timestamp", "x", "y", "z", "roll", "pitch", "yaw",
           "px", "py", "pz", "proll", "ppitch", "pyaw"
           ]

class RegulatorObserver:
    def __init__(self) -> None:
        print("Initializing regulator observer")
        self.log = np.empty((len(COLUMNS),))
        self.observing = False
        self.observation_start_time = None

        self.pub = rospy.Publisher(SET_POSITION_TOPIC, Twist, queue_size=10)
        msg = Twist()
        msg.linear = Vector3(0, 0, 0)
        msg.angular = Vector3(0, 0, 0)
        self.msg = msg

        rospy.init_node('RegulatorObserver', anonymous=True)
        self.rate = rospy.Rate(1)

    def log_position(self, data: Twist):
        if self.observing:
            timestamp = datetime.now().timestamp()
            position_vector = [data.linear.x, data.linear.y, data.linear.z]
            rotation_vector = [data.angular.x, data.angular.y, data.angular.z]

            predicted_position = [self.msg.linear.x, self.msg.linear.y, self.msg.linear.z]
            predicted_rotation = [self.msg.angular.x, self.msg.angular.y, self.msg.angular.z]

            if any(['nan' in str(x) for x in [*position_vector, *rotation_vector]]):
                self.end_observation()
            self.log = np.vstack((self.log, np.array([timestamp, *position_vector, *rotation_vector,
                                                      *predicted_position, *predicted_rotation])))

    def publish(self):
        self.pub.publish(self.msg)

    def listener(self):
        print("Starting ros subscriber")
        rospy.Subscriber(REGULATOR_POSITION_TOPIC, Twist, self.log_position)
        rospy.spin()

    def manage_observer(self):
        while not rospy.is_shutdown():
            self.publish()
            if self.observing:
                self.observe()
            else:
                self.check_requests()
            self.rate.sleep()

    def observe(self):
        time_elapsed: timedelta = datetime.now() - self.observation_start_time
        seconds_elapsed = time_elapsed.total_seconds()
        print(f"Observation time elapsed {seconds_elapsed}")

        new_pos = True
        if seconds_elapsed >= OBSERVATION_TIME / 2 and new_pos:
            self.msg.linear = Vector3(1, 0, 1)
            self.msg.angular = Vector3(0, math.pi / 2, 0)
            new_pos = False

        if seconds_elapsed >= OBSERVATION_TIME:
            self.end_observation()

    def end_observation(self):
        print("Finishing regulator observation")
        self.observing = False
        self.observation_start_time = None
        try:
            data = pd.DataFrame(self.log, columns=COLUMNS)
        except ValueError:
            data = pd.DataFrame(np.full((len(COLUMNS), 1), np.nan).transpose(), columns=COLUMNS)
        self.msg.linear = Vector3(0, 0, 0)
        self.msg.angular = Vector3(0, 0, 0)
        data.to_csv("observation_log.csv", index=False)
        print(f"Num of rows in data: {len(data)}")
        self.log = np.empty((len(COLUMNS),))

    def check_requests(self):
        mypath = os.getcwd()
        files = [f for f in os.listdir(mypath) if path.isfile(path.join(mypath, f))]
        for file in files:
            if file == "observation_request.txt":
                print("Found request file")
                os.remove(file)
                print("Starting regulator observation")
                self.observing = True
                self.observation_start_time = datetime.now()
                return


if __name__ == '__main__':
    regulator_observer = RegulatorObserver()
    print(f"Starting requests observation at {os.getcwd()}")
    worker = threading.Thread(target=regulator_observer.manage_observer)
    worker.start()
    regulator_observer.listener()
