#!/usr/bin/env python
import threading
from groundstation import webserver, pinger, rosnode, sshclient, database, state, logbook, systemdservices, rosrecording

luke_host = "128.199.39.87"
luke_user = "root"
luke_password = "fruasldufkhbukeasndfusa"

db = database.Database()
stat = state.StateManager(db)
ssh = sshclient.SSHClient(luke_host, stat, luke_user, luke_password)
pingrr = pinger.Pinger(luke_host, db, stat)
logbok = logbook.Logbook(db, stat)
websrver = webserver.Webserver(stat, logbok, ssh)
systemd = systemdservices.SystemdServices(ssh, stat)
rosrecorder = rosrecording.RosRecorder(stat, ssh)

ssh.start()
pingrr.start()
systemd.start()

webserverThread = threading.Thread(target=websrver.start)
webserverThread.daemon = True
webserverThread.start()

rosnode.RosNode(db, stat, logbok).run()
