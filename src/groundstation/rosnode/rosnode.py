import logging
import socket
import time

import rosgraph
import rospy
from cms.msg import Statistics, RecordingStatus, RecordingConfig
from cms.srv import RecordingUpdateConfig

from groundstation.rosnode.rosmeta import RosMeta
from groundstation.utils import add_logline, sendstate


class RosNode:

    def __init__(self, rosrecorder):
        self.statistics_subscriber = None
        self.recordingstatus_subscriber = None
        self.recordingconfig_subscriber = None
        self.update_recordingconfig_service = None
        self.rosbaginfo_service = None
        self.meta_timer = None
        self.rosmeta = RosMeta()
        self.rosrecorder = rosrecorder

    def run(self):
        while 1:
            self.set_rosnode_health(False)
            if rospy.is_shutdown():
                logging.info("Graceful end reached")
                return
            if self.is_master_disconnected():
                logging.info("master offline, retrying in 1 seconds")
                time.sleep(2)
                continue
            self.run_node()

    def run_node(self):
        rospy.init_node('cms')
        self.register_subscribers()
        self.register_timers()
        self.create_serviceproxies()
        self.set_rosnode_health(True)
        rate_1_second = rospy.Rate(2)
        while not rospy.is_shutdown():
            if self.is_master_disconnected():
                self.unregister_subscribers()
                self.unregister_timers()
                self.close_serviceproxies()
                return
            rate_1_second.sleep()

    def register_subscribers(self):
        self.statistics_subscriber = rospy.Subscriber("/cms/statistics", Statistics, self.rosmeta.statistics_callback)
        self.recordingstatus_subscriber = rospy.Subscriber("/cms/recording/status", RecordingStatus, self.rosrecorder.status_callback)
        self.recordingconfig_subscriber = rospy.Subscriber("/cms/recording/config", RecordingConfig, self.rosrecorder.config_callback)
        logging.info("Registered subscribers")

    def unregister_subscribers(self):
        self.statistics_subscriber.unregister()
        self.recordingstatus_subscriber.unregister()
        self.recordingconfig_subscriber.unregister()
        logging.info("Unregistered subscribers")

    def register_timers(self):
        logging.info("Registered timers")
        self.meta_timer = rospy.Timer(rospy.Duration(2), self.rosmeta.timercallback)

    def unregister_timers(self):
        logging.info("Unregistered timers")
        self.meta_timer.shutdown()

    def create_serviceproxies(self):
        self.update_recordingconfig_service = rospy.ServiceProxy('/cms/recording/updateconfig', RecordingUpdateConfig)
        self.rosbaginfo_service = rospy.ServiceProxy('/cms/recording/updateconfig', RecordingUpdateConfig)
        self.rosrecorder.set_service_proxies(self.update_recordingconfig_service, self.rosbaginfo_service)

    def close_serviceproxies(self):
        self.update_recordingconfig_service.close()
        self.rosbaginfo_service.close()

    def is_master_disconnected(self):
        try:
            rospy.get_master().getSystemState()
        except socket.error:
            return True
        return not rosgraph.is_master_online()

    def set_rosnode_health(self, up):
        sendstate({'rosnode_up': up})
