#!/usr/bin/env python
import threading
from groundstation import webserver, pinger, rosnode, sshclient, database, state, logbook, systemdservices, rosrecording

luke_host = "grace"
luke_user = "grace"
luke_password = ""

db = database.Database()
stat = state.StateManager(db)
ssh = sshclient.SSHClient(luke_host, stat, luke_user, luke_password)
pingrr = pinger.Pinger(luke_host, db, stat)
logbok = logbook.Logbook(db, stat)
rosrecorder = rosrecording.RosRecorder(stat, ssh)
websrver = webserver.Webserver(stat, logbok, ssh, rosrecorder)
systemd = systemdservices.SystemdServices(ssh, stat)

ssh.start()
pingrr.start()
systemd.start()
rosrecorder.start()

webserverThread = threading.Thread(target=websrver.start)
webserverThread.daemon = True
webserverThread.start()

rosnode.RosNode(db, stat, logbok).run()
