#!/usr/bin/env python
import rospy
import json
from typing import *
from collections import deque # Python Queue class
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist


NODE_NAME = "TrajectoryPlanner"
CONFIG_FILE_PATH = '/'.join([*__file__.split('/')[0:-3], 'auvConfig', 'auvConfig.json'])
HEALTH_REPORT_TOPIC = "/AUVInternalSystem/Health/healthReportSingleNode"


class TrajectoryPlanner:
    def __init__(self) -> None:
        # FIELD DECLARATIONS
        self.nodeID: int = 6
        self.ticks: int = 0
        
        self.position: Twist = None
        self.trajectory: Deque[Twist] = deque()


        # INIT METHODS
        rospy.init_node(NODE_NAME, anonymous=True)
        with open(CONFIG_FILE_PATH, 'r') as file:
            json_data: Any = json.load(file)

            rosRate = json_data["ROS"]["rate"]
            self.rosRate: rospy.Rate = rospy.Rate(rosRate)
            self.healthReportRate: int = json_data["ROS"]["healthReportRate"]
            self.healthReportTickSpan: float = rosRate / self.healthReportRate
        
        self.healthReportPublisher: rospy.Publisher = rospy.Publisher(HEALTH_REPORT_TOPIC, Int32, queue_size=10)

        self.subscribe_topics()
        self.advertise_topics()

        self.run()

    def subscribe_topics(self) -> None:
        pass

    def advertise_topics(self) -> None:
        pass
    
    def report_health(self) -> None:
        healthMsg: Int32 = Int32()
        healthMsg.data = 1 << self.nodeID

        self.healthReportPublisher.publish(healthMsg)

    def run(self):
        while not rospy.is_shutdown():
            if self.ticks % self.healthReportTickSpan == 0:
                self.report_health()
            self.ticks += 1

            self.rosRate.sleep()

if __name__ == '__main__':
    try:
        trajectory_planner = TrajectoryPlanner()
        
    except rospy.ROSInterruptException:
        pass
