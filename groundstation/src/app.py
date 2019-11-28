#!/usr/bin/env python
import threading
import webserver
import rosnode
import pinger
import sshclient

pinger.initialize()
sshclient.initialize()

webserverThread = threading.Thread(target=webserver.start_server)
webserverThread.daemon = True
webserverThread.start()

rosnode.run()
