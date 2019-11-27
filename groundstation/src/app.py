#!/usr/bin/env python
import threading
import webserver
import rosnode
import pinger
import sshclient


webserverThread = threading.Thread(target=webserver.start_server)
webserverThread.daemon = True
webserverThread.start()

pinger.initialize()
sshclient.initialize()

rosnode.run()
