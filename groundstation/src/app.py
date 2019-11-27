#!/usr/bin/env python
import threading
import webserver
import rosnode


webserverThread = threading.Thread(target=webserver.start_server)
webserverThread.daemon = True
webserverThread.start()

rosnode.start_server()
