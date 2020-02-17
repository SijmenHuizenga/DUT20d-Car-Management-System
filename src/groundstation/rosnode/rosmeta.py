import logging
import time

import rospy

from groundstation.models import TopicStatistic, Topic, Node, TopicSubscription, TopicPublication, TopicType
from groundstation.utils import is_topic_ignored, is_node_ignored, sendstate


class RosMeta:
    def __init__(self):
        self.nodes = []
        self.topics = []
        self.subscriptions = []
        self.publications = []
        self.topictypes = []
        pass

    def timercallback(self, event):
        try:
            self.update_topictypes()
            sendstate({'topictypes': self.topictypes})

            self.update_pubsubs()
            sendstate({
                'nodes': self.nodes,
                'topics': self.topics,
                'subscriptions': self.subscriptions,
                'publications': self.publications,
            })
        except Exception, e:
            logging.error(e)
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
        for s in self.subscriptions:
            if s.topicname == topic and s.nodename == node:
                s.lastseen = lastseen
                break
        else:
            self.subscriptions.append(TopicSubscription(node, topic, lastseen))

    def process_publications(self, publications, lastseen):
        for (topic, nodes) in publications:
            for node in nodes:
                self.process_publication(topic, node, lastseen)

    def process_publication(self, topic, node, lastseen):
        self.update_topic(topic, lastseen)
        self.update_node(node, lastseen)
        if is_topic_ignored(topic) or is_node_ignored(node):
            return
        for s in self.publications:
            if s.topicname == topic and s.nodename == node:
                s.lastseen = lastseen
                break
        else:
            self.publications.append(TopicPublication(node, topic, lastseen))

    def update_topictypes(self):
        code, msg, types = rospy.get_master().getTopicTypes()
        now = time.time()
        if code != 1:
            raise Exception("rosmaster.getSystemState failed: ", code, msg)
        for (topic, messagetype) in types:
            if is_topic_ignored(topic):
                continue
            for s in self.topictypes:
                if s.topicname == topic:
                    s.lastseen = now
                    s.messagetype = messagetype
                    break
            else:
                self.topictypes.append(TopicType(topic, now, messagetype))

    def update_topic(self, topic, lastseen):
        if is_topic_ignored(topic):
            return
        for s in self.topics:
            if s.name == topic:
                if s.lastseen < lastseen:
                    s.lastseen = lastseen
                break
        else:
            self.topics.append(Topic(topic, lastseen, statistics=None))

    def update_node(self, node, lastseen):
        if is_node_ignored(node):
            return
        for s in self.nodes:
            if s.name == node:
                if s.lastseen < lastseen:
                    s.lastseen = lastseen
                break
        else:
            self.nodes.append(Node(node, lastseen))

    def statistics_callback(self, statisticsmessage):
        for topicstats in statisticsmessage.statistics:
            self.update_statistics(topicstats)
        sendstate({'topics': self.topics})

    def update_statistics(self, topicstats):
        if is_topic_ignored(topicstats.topic_name):
            return
        for existingTopic in self.topics:
            if existingTopic.name == topicstats.topic_name:
                existingTopic.statistics = TopicStatistic(lastseen=time.time(), traffic=topicstats.traffic)
                break
        else:
            self.topics.append(Topic(
                name=topicstats.topic_name,
                lastseen=time.time(),
                statistics=TopicStatistic(
                    lastseen=time.time(),
                    traffic=topicstats.traffic)
            ))
