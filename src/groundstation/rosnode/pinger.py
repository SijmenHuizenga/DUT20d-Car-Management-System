import logging
import os
import threading
import time

from groundstation.models import Ping
from groundstation.utils import sendstate


class Pinger:
    def __init__(self, host):
        self.host = host

    def start(self):
        thread = threading.Thread(target=self.ping_forever)
        thread.daemon = True
        thread.start()

    def ping_forever(self):
        while 1:
            try:
                self.ping()
            except Exception, e:
                logging.error("Someting unexpected happend while pinging %s: %s" % (self.host, str(e)))
            time.sleep(0.7)

    def ping(self):
        success = os.system("ping -c 1 -w 1 -W 1 %s > /dev/null 2>&1" % self.host) == 0
        sendstate({'ping': Ping(time.time(), success)})
