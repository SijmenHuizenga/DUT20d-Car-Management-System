#!/usr/bin/env python
import threading
from groundstation import webserver, pinger, rosnode, sshclient, database, state, logbook, systemdservices, rosrecording

luke_host = "192.168.33.1"
luke_user = "luke"
luke_password = ""

database_ = database.Database()
state_ = state.State(database_)
sshclient_ = sshclient.SSHClient(luke_host, state_, luke_user, luke_password)
pinger_ = pinger.Pinger(luke_host, state_)
logbook_ = logbook.Logbook(database_, state_)
rosrecorder_ = rosrecording.RosRecorder(state_, sshclient_, logbook_)
webserver_ = webserver.Webserver(state_, logbook_, sshclient_, rosrecorder_)
systemdservices_ = systemdservices.SystemdServices(sshclient_, state_)

sshclient_.start()
pinger_.start()
systemdservices_.start()

webserverThread = threading.Thread(target=webserver_.start)
webserverThread.daemon = True
webserverThread.start()

state_.start()

rosnode.RosNode(database_, state_, logbook_, rosrecorder_).run()
