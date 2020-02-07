import os
import threading
import time

from .state import State


class Pinger:
    def __init__(self,
                 host,  # type: str
                 state  # type: State
                 ):
        self.host = host
        self.state = state

    def start(self):
        thread = threading.Thread(target=self.ping_forever)
        thread.daemon = True
        thread.start()

    def ping_forever(self):
        while 1:
            try:
                self.ping()
            except Exception, e:
                print("[pinger] Someting unexpected happend while pinging %s: %s" % (self.host, str(e)))
            time.sleep(0.7)

    def ping(self):
        success = os.system("ping -c 1 -w 1 -W 1 %s > /dev/null 2>&1" % self.host) == 0

        now = time.time()
        self.state.ping.timestamp = now
        self.state.ping.success = success
        self.state.emit_state()
