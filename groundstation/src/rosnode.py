#!/usr/bin/env python
import rospy
import time
import socket
from std_msgs.msg import String
import rosgraph
from state import statemanager as state


global subscrib


def register_subscribers():
    global subscrib
    subscrib = rospy.Subscriber("chatter", String, callback)
    print("[rosnode] Registered subscribers")


def unregister_subscribers():
    global subscrib
    subscrib.unregister()
    print("[rosnode] Unregistered subscribers")


def callback(msg):
    print(msg)


def is_master_disconnected():
    try:
        rospy.get_master().getSystemState()
    except socket.error:
        return True
    return not rosgraph.is_master_online()


def run_node():
    rospy.init_node('cms_groundstation')
    register_subscribers()
    state.set_rosnode_health(True)
    rate_1_second = rospy.Rate(1)
    while 1:
        if rospy.is_shutdown():
            return
        if is_master_disconnected():
            unregister_subscribers()
            return
        rate_1_second.sleep()


def run():
    while 1:
        state.set_rosnode_health(False)
        if rospy.is_shutdown():
            print("[rosnode] Graceful end reached")
            return
        if is_master_disconnected():
            print("[rosnode] master offline, retrying in 1 seconds")
            time.sleep(1)
            continue
        run_node()
