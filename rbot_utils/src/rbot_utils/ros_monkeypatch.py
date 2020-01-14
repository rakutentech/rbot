#! /usr/bin/env python

from __future__ import print_function

import rospy
import pytest


class RospySubscriber():
    def __init__(self, topic, topic_type, callback):
        self.topic = topic
        self.topic_type = topic_type
        self.callback = callback
        self.active = True
        self.activated = 0

    def unregister(self):
        self.active = False

    def activate(self, msg):
        self.activated += 1
        self.callback(msg)


class RospyPublisher():
    def __init__(self, topic, topic_type, queue_size=10):
        self.topic = topic
        self.topic_type = topic_type
        self.activated = 0

    def publish(self, message):
        self.activated += 1


class RospyService():
    def __init__(self, topic, topic_type, callback):
        self.topic = topic
        self.topic_type = topic_type
        self.callback = callback
        self.activated = 0

    def activate(self, msg):
        self.activated += 1
        return self.callback(msg)


@pytest.fixture(autouse=True)
def no_rospy(monkeypatch):
    monkeypatch.setattr(rospy, "logdebug", print)
    monkeypatch.setattr(rospy, "loginfo", print)
    monkeypatch.setattr(rospy, "logwarn", print)
    monkeypatch.setattr(rospy, "logfatal", print)

    monkeypatch.setattr(rospy, "Duration", lambda x: x)
    monkeypatch.setattr(rospy, "get_rostime", lambda: 34)
    monkeypatch.setattr(rospy, "Rate", lambda x: x)

    monkeypatch.setattr(rospy, "Publisher", RospyPublisher)
    monkeypatch.setattr(rospy, "Subscriber", RospySubscriber)
    monkeypatch.setattr(rospy, "Service", RospyService)
