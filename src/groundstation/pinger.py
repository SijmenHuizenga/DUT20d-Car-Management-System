import os
import threading
import time
from database import database as db
from state import statemanager as state


class Pinger:
    def __init__(self, devicename, deviceaddress):
        self.devicename = devicename
        self.deviceaddress = deviceaddress

        thread = threading.Thread(target=self.ping_forever)
        thread.daemon = True
        thread.start()

    def ping_forever(self):
        while 1:
            try:
                self.ping()
            except Exception, e:
                print("[pinger] Someting unexpected happend while pinging the %s: %s" % (self.devicename, str(e)))
                pass
            time.sleep(1)

    def ping(self):
        response = os.system("ping -c 1 -w 1 -W 1 %s > /dev/null 2>&1" % self.deviceaddress)
        success = response == 0

        now = time.time()
        db.insert('INSERT INTO pings VALUES (:now, :device, :success)',
                  {'now': now, 'device': self.devicename, 'success': success})

        state.update({
            'pinger': {
                self.devicename: {
                    'timestamp': now,
                    'success': success,
                }
            },
        })
