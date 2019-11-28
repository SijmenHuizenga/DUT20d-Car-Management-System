#!/usr/bin/env python
import threading
from groundstation import rosnode
from groundstation import webserver
from groundstation import pinger

pinger.initialize()

webserverThread = threading.Thread(target=webserver.start_server)
webserverThread.daemon = True
webserverThread.start()

rosnode.run()
