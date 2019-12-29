import socket
import time

import rosgraph
import rospy

from .database import Database
from .logbook import Logbook
from .state import State, TopicType, Topic, TopicSubscription, Node, TopicPublication


class RosNode:

    def __init__(self,
                 db,  # type: Database
                 state,  # type: State
                 logbook  # type: Logbook
                 ):
        self.subscrib = None
        self.rosmeta = RosMeta(db, state)
        self.db = db
        self.state = state
        self.logbook = logbook

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
            self.logbook.add_line(time.time(), "Disconnected from ROS master", "groundstation")

    def run_node(self):
        rospy.init_node('cms')
        self.register_subscribers()
        self.register_timers()
        self.set_rosnode_health(True)
        self.logbook.add_line(time.time(), "Connected to ROS master", "groundstation")
        rate_1_second = rospy.Rate(1)
        while not rospy.is_shutdown():
            if self.is_master_disconnected():
                self.unregister_subscribers()
                return
            rate_1_second.sleep()

    def register_subscribers(self):
        print("[rosnode] Registered subscribers")

    def unregister_subscribers(self):
        self.subscrib.unregister()
        print("[rosnode] Unregistered subscribers")

    def register_timers(self):
        rospy.Timer(rospy.Duration(3), self.rosmeta.timercallback)

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
        self.state.emit_state()


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
            # todo: also print stacktrace
            raise

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
        for s in self.state.subscriptions:
            if s.topicname == topic and s.nodename == node:
                s.lastseen = lastseen
                break
        else:
            self.state.subscriptions.append(TopicSubscription(node, topic, lastseen))

        self.update_topic(topic, lastseen)
        self.update_node(node, lastseen)

    def process_publications(self, publications, lastseen):
        for (topic, nodes) in publications:
            for node in nodes:
                self.process_publication(topic, node, lastseen)

    def process_publication(self, topic, node, lastseen):
        for s in self.state.publications:
            if s.topicname == topic and s.nodename == node:
                s.lastseen = lastseen
                break
        else:
            self.state.publications.append(TopicPublication(node, topic, lastseen))

        self.update_topic(topic, lastseen)
        self.update_node(node, lastseen)

    def update_topictypes(self):
        code, msg, types = rospy.get_master().getTopicTypes()
        now = time.time()
        if code != 1:
            raise Exception("rosmaster.getSystemState failed: ", code, msg)
        for (topic, messagetype) in types:
            for s in self.state.topictypes:
                if s.topicname == topic:
                    s.lastseen = now
                    s.messagetype = messagetype
                    break
            else:
                self.state.topictypes.append(TopicType(topic, now, messagetype))
            self.update_topic(topic, now)

    def update_topic(self, topic, lastseen):
        for s in self.state.topics:
            if s.name == topic:
                if s.lastseen < lastseen:
                    s.lastseen = lastseen
                break
        else:
            self.state.topics.append(Topic(topic, lastseen))

    def update_node(self, node, lastseen):
        for s in self.state.nodes:
            if s.name == node:
                if s.lastseen < lastseen:
                    s.lastseen = lastseen
                break
        else:
            self.state.nodes.append(Node(node, lastseen))
