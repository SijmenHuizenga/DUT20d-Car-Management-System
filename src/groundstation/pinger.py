import os
import threading
import time
from database import database as db
from state import statemanager as state


class Pinger:
    def __init__(self, host):
        self.host = host

        thread = threading.Thread(target=self.ping_forever)
        thread.daemon = True
        thread.start()

    def ping_forever(self):
        while 1:
            try:
                self.ping()
            except Exception, e:
                print("[pinger] Someting unexpected happend while pinging %s: %s" % (self.host, str(e)))
            time.sleep(1)

    def ping(self):
        success = os.system("ping -c 1 -w 1 -W 1 %s > /dev/null 2>&1" % self.host) == 0

        now = time.time()
        db.insert('INSERT INTO pings VALUES (:now, :host, :success)',
                  {'now': now, 'host': self.host, 'success': success})

        state.update({
            'ping': {
                'timestamp': now,
                'success': success,
            },
        })
