import rospy
import time
import socket
import std_msgs.msg
import rosgraph


class RosNode:

    def __init__(self, db, state, logbook):
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
        while 1:
            if rospy.is_shutdown():
                return
            if self.is_master_disconnected():
                self.unregister_subscribers()
                return
            rate_1_second.sleep()

    def register_subscribers(self):
        self.subscrib = rospy.Subscriber("chatter", std_msgs.msg.String, self.callback)
        print("[rosnode] Registered subscribers")

    def unregister_subscribers(self):
        self.subscrib.unregister()
        print("[rosnode] Unregistered subscribers")

    def register_timers(self):
        rospy.Timer(rospy.Duration(3), self.rosmeta.timercallback)

    def callback(self, msg):
        print('callback', msg)

    def is_master_disconnected(self):
        try:
            rospy.get_master().getSystemState()
        except socket.error:
            return True
        return not rosgraph.is_master_online()

    def set_rosnode_health(self, up):
        self.db.insert('INSERT INTO rosnodehealth VALUES (:now, :up)',
                  {'now': time.time(), 'up': up})
        self.state.update({'rosnode': {'up': up}})


class RosMeta:
    def __init__(self, db, state):
        self.db = db
        self.state = state
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
            # todo: also print stacktrace
            print('rosmeta exception', e)

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
                        'subscriptions': [topic],
                        'publications': []
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
                        'publications': [topic],
                        'subscriptions': []
                    }
            if topic not in all_topics:
                all_topics[topic] = {
                    'lastseen': lastseen,
                    'type': 'unknown'
                }

        return all_topics, all_nodes

    def store_topics(self, lastseen, all_topics):
        for topicname in all_topics:
            self.db.insert("INSERT INTO topics VALUES (:lastseen, :name, :type)",
                           {'lastseen': lastseen, 'name': topicname, 'type': all_topics[topicname]['type']})

    def store_nodes(self, lastseen, all_nodes):
        for nodename in all_nodes:
            self.db.insert("INSERT INTO nodes VALUES (:lastseen, :name)",
                           {'lastseen': lastseen, 'name': nodename})
            for topicname in all_nodes[nodename]['subscriptions']:
                self.db.insert("INSERT INTO node_topic_subscription VALUES (:lastseen, :node, :topic)",
                               {'lastseen': lastseen, 'node': nodename, 'topic': topicname})
            for topicname in all_nodes[nodename]['publications']:
                self.db.insert("INSERT INTO node_topic_publication VALUES (:lastseen, :node, :topic)",
                               {'lastseen': lastseen, 'node': nodename, 'topic': topicname})

    def update_topics_state(self, all_topics):
        topicstate = self.state.state['topics']
        for topicname in all_topics:
            topicstate[topicname] = all_topics[topicname]

    def update_nodes_state(self, all_nodes):
        nodestate = self.state.state['nodes']
        for nodename in all_nodes:
            nodestate[nodename] = all_nodes[nodename]