import socket
import time

import rosgraph
import rospy
from cms.msg import Statistics, RecordingStatus, RecordingConfig
from cms.srv import RecordingUpdateConfig

from .rosrecording import RosRecorder
from .database import Database
from .logbook import Logbook
from .state import State, TopicType, Topic, TopicSubscription, Node, TopicPublication, TopicStatistic


class RosNode:

    def __init__(self,
                 db,  # type: Database
                 state,  # type: State
                 logbook,  # type: Logbook
                 rosrecorder,  # type: RosRecorder
                 ):
        self.statistics_subscriber = None
        self.recordingstatus_subscriber = None
        self.recordingconfig_subscriber = None
        self.update_recordingconfig_service = None
        self.rosbaginfo_service = None
        self.meta_timer = None
        self.rosmeta = RosMeta(db, state)
        self.db = db
        self.state = state
        self.logbook = logbook
        self.rosrecorder = rosrecorder

    def run(self):
        while 1:
            self.set_rosnode_health(False)
            if rospy.is_shutdown():
                print("[rosnode] Graceful end reached")
                return
            if self.is_master_disconnected():
                print("[rosnode] master offline, retrying in 1 seconds")
                time.sleep(2)
                continue
            self.run_node()
            self.logbook.add_line(time.time(), "Disconnected from ROS master", "groundstation")

    def run_node(self):
        rospy.init_node('cms')
        self.register_subscribers()
        self.register_timers()
        self.create_serviceproxies()
        self.set_rosnode_health(True)
        self.logbook.add_line(time.time(), "Connected to ROS master", "groundstation")
        rate_1_second = rospy.Rate(2)
        while not rospy.is_shutdown():
            if self.is_master_disconnected():
                self.unregister_subscribers()
                self.unregister_timers()
                self.close_serviceproxies()
                return
            rate_1_second.sleep()

    def register_subscribers(self):
        self.statistics_subscriber = rospy.Subscriber("/cms/statistics", Statistics, self.statistics_callback)
        self.recordingstatus_subscriber = rospy.Subscriber("/cms/recording/status", RecordingStatus, self.rosrecorder.status_callback)
        self.recordingconfig_subscriber = rospy.Subscriber("/cms/recording/config", RecordingConfig, self.rosrecorder.config_callback)
        print("[rosnode] Registered subscribers")

    def unregister_subscribers(self):
        self.statistics_subscriber.unregister()
        self.recordingstatus_subscriber.unregister()
        self.recordingconfig_subscriber.unregister()
        print("[rosnode] Unregistered subscribers")

    def register_timers(self):
        print("[rosnode] Registered timers")
        self.meta_timer = rospy.Timer(rospy.Duration(2), self.rosmeta.timercallback)

    def unregister_timers(self):
        print("[rosnode] Unregistered timers")
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
        self.db.insert('INSERT INTO rosnodehealth VALUES (:now, :up)',
                       {'now': time.time(), 'up': up})
        self.state.rosnode.up = up

    def statistics_callback(self, statisticsmessage):
        for topicstats in statisticsmessage.statistics:
            self.update_statistics(topicstats)

    def update_statistics(self, topicstats):
        if is_topic_ignored(topicstats.topic_name):
            return
        for existingTopic in self.state.topics:
            if existingTopic.name == topicstats.topic_name:
                existingTopic.statistics = TopicStatistic(lastseen=time.time(), traffic=topicstats.traffic)
                break
        else:
            self.state.topics.append(Topic(
                name=topicstats.topic_name,
                lastseen=time.time(),
                statistics=TopicStatistic(
                    lastseen=time.time(),
                    traffic=topicstats.traffic)
            ))

class RosMeta:
    def __init__(self,
                 db,  # type: Database
                 state,  # type: State
                 ):
        self.db = db
        self.state = state
        pass

    def timercallback(self, event):
        try:
            self.update_topictypes()
            self.update_pubsubs()
        except Exception, e:
            print e
            # todo: print stacktrace
            return

    def update_pubsubs(self):
        code, msg, val = rospy.get_master().getSystemState()
        now = time.time()
        if code != 1:
            raise Exception("rosmaster.getSystemState failed: ", code, msg)
        publishers, subscribers, srvs = val
        self.process_subscriptions(subscribers, now)
        self.process_publications(publishers, now)

    def process_subscriptions(self, subscribers, lastseen):
        for (topic, nodes) in subscribers:
            for node in nodes:
                self.process_subscription(topic, node, lastseen)

    def process_subscription(self, topic, node, lastseen):
        self.update_topic(topic, lastseen)
        self.update_node(node, lastseen)

        if is_topic_ignored(topic) or is_node_ignored(node):
            return
        for s in self.state.subscriptions:
            if s.topicname == topic and s.nodename == node:
                s.lastseen = lastseen
                break
        else:
            self.state.subscriptions.append(TopicSubscription(node, topic, lastseen))

    def process_publications(self, publications, lastseen):
        for (topic, nodes) in publications:
            for node in nodes:
                self.process_publication(topic, node, lastseen)

    def process_publication(self, topic, node, lastseen):
        self.update_topic(topic, lastseen)
        self.update_node(node, lastseen)
        if is_topic_ignored(topic) or is_node_ignored(node):
            return
        for s in self.state.publications:
            if s.topicname == topic and s.nodename == node:
                s.lastseen = lastseen
                break
        else:
            self.state.publications.append(TopicPublication(node, topic, lastseen))

    def update_topictypes(self):
        code, msg, types = rospy.get_master().getTopicTypes()
        now = time.time()
        if code != 1:
            raise Exception("rosmaster.getSystemState failed: ", code, msg)
        for (topic, messagetype) in types:
            if is_topic_ignored(topic):
                continue
            for s in self.state.topictypes:
                if s.topicname == topic:
                    s.lastseen = now
                    s.messagetype = messagetype
                    break
            else:
                self.state.topictypes.append(TopicType(topic, now, messagetype))
            self.update_topic(topic, now)

    def update_topic(self, topic, lastseen):
        if is_topic_ignored(topic):
            return
        for s in self.state.topics:
            if s.name == topic:
                if s.lastseen < lastseen:
                    s.lastseen = lastseen
                break
        else:
            self.state.topics.append(Topic(topic, lastseen, statistics=None))

    def update_node(self, node, lastseen):
        if is_node_ignored(node):
            return
        for s in self.state.nodes:
            if s.name == node:
                if s.lastseen < lastseen:
                    s.lastseen = lastseen
                break
        else:
            self.state.nodes.append(Node(node, lastseen))


def is_topic_ignored(topicname):
    if topicname.startswith("/rosout"):
        return True
    return False


def is_node_ignored(nodename):
    if nodename.startswith("/ros"):
        return True
    return False
