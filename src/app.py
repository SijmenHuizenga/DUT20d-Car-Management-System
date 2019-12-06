#!/usr/bin/env python
import threading
from groundstation import sshclient

luke_host = "grace"
luke_user = "grace"
luke_password = ""
sshclient.init(luke_host, luke_user, luke_password)

from groundstation import webserver, pinger, rosnode


pinger.Pinger(luke_host)

webserverThread = threading.Thread(target=webserver.start_server)
webserverThread.daemon = True
webserverThread.start()

rosnode.RosNode().run()
