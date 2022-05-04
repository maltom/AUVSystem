#!/usr/bin/env python
import os
import os.path as path
import rospy
import threading
import numpy as np
import pandas as pd
from datetime import datetime, timedelta
from geometry_msgs.msg import Twist

REGULATOR_POSITION_TOPIC = "/AUVInternalSystem/Position/globalEstimatedPosition"
OBSERVATION_TIME = 30

class RegulatorObserver:
    def __init__(self) -> None:
        print("Initializing regulator observer")
        self.log = np.empty((7,))
        self.observing = False
        self.observation_start_time = None
        rospy.init_node('RegulatorObserver', anonymous=True)
        self.rate = rospy.Rate(1)

    def log_position(self, data: Twist):
        if self.observing:
            timestamp = datetime.now().timestamp()
            position_vector = [data.linear.x, data.linear.y, data.linear.z]
            rotation_vector = [data.angular.x, data.angular.y, data.angular.z]
            if any(['nan' in str(x) for x in [*position_vector, *rotation_vector]]):
                self.end_observation()
            self.log = np.vstack((self.log, np.array([timestamp, *position_vector, *rotation_vector])))

    def listener(self):
        print("Starting ros subscriber")
        rospy.Subscriber(REGULATOR_POSITION_TOPIC, Twist, self.log_position)
        rospy.spin()

    def manage_observer(self):
        while not rospy.is_shutdown():
            if self.observing:
                self.observe()
            else:
                self.check_requests()
            self.rate.sleep()

    def observe(self):
        time_elapsed: timedelta = datetime.now() - self.observation_start_time
        seconds_elapsed = time_elapsed.total_seconds()
        print(f"Observation time elapsed {seconds_elapsed}")
        if seconds_elapsed >= 30:
            self.end_observation()

    def end_observation(self):
        print("Finishing regulator observation")
        self.observing = False
        self.observation_start_time = None
        try:
            data = pd.DataFrame(self.log, columns=["timestamp", "x", "y", "z", "roll", "pitch", "yaw"])
        except ValueError:
            data = pd.DataFrame(np.full((7, 1), np.nan).transpose(), columns=["timestamp", "x", "y", "z", "roll", "pitch", "yaw"])
        data.to_csv("observation_log.csv", index=False)
        self.log = np.empty((7,))

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
