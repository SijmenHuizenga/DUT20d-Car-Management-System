import rospy
import time
import socket
from std_msgs.msg import String
import rosgraph
from database import database as db
from state import statemanager as state
from logbook import logbook
from rosmeta import RosMeta


class RosNode:

    def __init__(self):
        self.subscrib = None
        self.rosmeta = RosMeta()

    def run(self):
        while 1:
            self.set_rosnode_health(False)
            if rospy.is_shutdown():
                print("[rosnode] Graceful end reached")
                return
            if self.is_master_disconnected():
                print("[rosnode] master offline, retrying in 1 seconds")
                time.sleep(1)
                continue
            self.run_node()
            logbook.add_line(time.time(), "Disconnected from ROS master", "groundstation")

    def run_node(self):
        rospy.init_node('cms')
        self.register_subscribers()
        self.register_timers()
        self.set_rosnode_health(True)
        logbook.add_line(time.time(), "Connected to ROS master", "groundstation")
        rate_1_second = rospy.Rate(1)
        while 1:
            if rospy.is_shutdown():
                return
            if self.is_master_disconnected():
                self.unregister_subscribers()
                return
            rate_1_second.sleep()

    def register_subscribers(self):
        self.subscrib = rospy.Subscriber("chatter", String, self.callback)
        print("[rosnode] Registered subscribers")

    def unregister_subscribers(self):
        self.subscrib.unregister()
        print("[rosnode] Unregistered subscribers")

    def register_timers(self):
        rospy.Timer(rospy.Duration(3), self.rosmeta.timercallback)

    def callback(self, msg):
        print('callback', msg)

    @staticmethod
    def is_master_disconnected():
        try:
            rospy.get_master().getSystemState()
        except socket.error:
            return True
        return not rosgraph.is_master_online()

    @staticmethod
    def set_rosnode_health(up):
        db.insert('INSERT INTO rosnodehealth VALUES (:now, :up)',
                  {'now': time.time(), 'up': up})
        state.update({'rosnode': {'up': up}})

