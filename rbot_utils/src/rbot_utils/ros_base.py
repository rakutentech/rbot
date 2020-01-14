#! /usr/bin/env python

"""
Ensures that ROS node is well up and running to ensure no spurious failures
take place in inter-process-comunication
"""


class RosBase(object):
    def __init__(self):
        from time import sleep
        from rospy import get_time

        while get_time() == 0:
            sleep(0.1)
