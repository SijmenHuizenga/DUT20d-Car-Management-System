import logging
import threading
import time

from groundstation.utils import sendstate


class SSHPing:

    def __init__(self, client):
        self.client = client

    def start(self):
        thread = threading.Thread(target=self.forever)
        thread.daemon = True
        thread.start()

    def forever(self):
        while 1:
            try:
                self.ping()
            except Exception, e:
                logging.error("[sshclient] Someting unexpected happend while checking health: %s" % str(e))
            time.sleep(1.25)

    def ping(self):
        try:
            self.client.ensure_transport()
            (exitcode, uptimestr) = self.client.run_command("uptime -p")
            if exitcode != 0:
                raise Exception("uptime exit code was not 0")
            connected = True
        except Exception, e:
            uptimestr = str(e)
            connected = False

        sendstate({'ssh': {
            'connected': connected,
            'uptime': uptimestr,
            'lastping': time.time()
        }})
