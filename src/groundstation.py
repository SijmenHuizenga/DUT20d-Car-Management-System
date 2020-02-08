#!/usr/bin/env python

# Initializes the eventlet async webserver.
# This is required for multithreaded message emitting over websocket.
# Needs to be the first import in the whole program. Read more about it here:
#  https://eventlet.net/doc/patching.html
import eventlet
eventlet.monkey_patch()

import threading
from groundstation import webserver, pinger, rosnode, sshclient, state, systemdservices, rosrecording

luke_host = "192.168.33.1"
luke_user = "luke"
luke_password = ""

state_ = state.State()
sshclient_ = sshclient.SSHClient(luke_host, state_, luke_user, luke_password)
pinger_ = pinger.Pinger(luke_host, state_)
rosrecorder_ = rosrecording.RosRecorder(state_, sshclient_)
webserver_ = webserver.Webserver(state_, sshclient_, rosrecorder_)
systemdservices_ = systemdservices.SystemdServices(sshclient_, state_)

sshclient_.start()
pinger_.start()
systemdservices_.start()

webserverThread = threading.Thread(target=webserver_.start)
webserverThread.daemon = True
webserverThread.start()

state_.start()

rosnode.RosNode(state_, rosrecorder_).run()
