import time

import rospy
from database import database as db
from state import statemanager as state


class RosMeta:
    def __init__(self):
        pass

    def timercallback(self, event):
        try:
            code, msg, val = rospy.get_master().getSystemState()
            if code != 1:
                raise Exception("rosmaster.getSystemState failed: ", code, msg)
            pubs, subs, srvs = val

            code, msg, types = rospy.get_master().getTopicTypes()
            if code != 1:
                raise Exception("rosmaster.getSystemState failed: ", code, msg)
            now = time.time()
            all_topics, all_nodes = self.process_new_rosinformation(now, pubs, subs, types)
            self.store_topics(now, all_topics)
            self.store_nodes(now, all_nodes)

            self.update_topics_state(all_topics)
            self.update_nodes_state(all_nodes)

        except Exception, e:
            print(e)

    def process_new_rosinformation(self, lastseen, publishers, subscriptions, topictypes):
        all_topics = {}
        all_nodes = {}

        for (topic, messagetype) in topictypes:
            all_topics[topic] = {
                'lastseen': lastseen,
                'type': messagetype
            }

        for (topic, nodes) in subscriptions:
            for node in nodes:
                if node in all_nodes:
                    if 'subscriptions' in all_nodes[node]:
                        all_nodes[node]['subscriptions'].append(topic)
                    else:
                        all_nodes[node]['subscriptions'] = [topic]
                else:
                    all_nodes[node] = {
                        'lastseen': lastseen,
                        'subscriptions': [topic]
                    }
            if topic not in all_topics:
                all_topics[topic] = {
                    'lastseen': lastseen,
                    'type': 'unknown'
                }

        for (topic, nodes) in publishers:
            for node in nodes:
                if node in all_nodes:
                    if 'publications' in all_nodes[node]:
                        all_nodes[node]['publications'].append(topic)
                    else:
                        all_nodes[node]['publications'] = [topic]
                else:
                    all_nodes[node] = {
                        'lastseen': lastseen,
                        'publications': [topic]
                    }
            if topic not in all_topics:
                all_topics[topic] = {
                    'lastseen': lastseen,
                    'type': 'unknown'
                }

        return all_topics, all_nodes

    def store_topics(self, lastseen, all_topics):
        for topicname in all_topics:
            db.insert("INSERT INTO topics VALUES (:lastseen, :name, :type)",
                      {'lastseen': lastseen, 'name': topicname, 'type': all_topics[topicname]['type']})

    def store_nodes(self, lastseen, all_nodes):
        for nodename in all_nodes:
            db.insert("INSERT INTO nodes VALUES (:lastseen, :name)",
                      {'lastseen': lastseen, 'name': nodename})
            for topicname in all_nodes[nodename]['subscriptions']:
                db.insert("INSERT INTO node_topic_subscription VALUES (:lastseen, :node, :topic)",
                          {'lastseen': lastseen, 'node': nodename, 'topic': topicname})
            for topicname in all_nodes[nodename]['publications']:
                db.insert("INSERT INTO node_topic_publication VALUES (:lastseen, :node, :topic)",
                          {'lastseen': lastseen, 'node': nodename, 'topic': topicname})

    def update_topics_state(self, all_topics):
        topicstate = state.state['topics']
        for topicname in all_topics:
            topicstate[topicname] = all_topics[topicname]

    def update_nodes_state(self, all_nodes):
        nodestate = state.state['nodes']
        for nodename in all_nodes:
            nodestate[nodename] = all_nodes[nodename]
