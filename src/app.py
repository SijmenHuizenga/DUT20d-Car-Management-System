#!/usr/bin/env python
import threading
from groundstation import webserver, pinger, rosnode

pinger.Pinger("computebox", "google.com")

webserverThread = threading.Thread(target=webserver.start_server)
webserverThread.daemon = True
webserverThread.start()

rosnode.RosNode().run()
