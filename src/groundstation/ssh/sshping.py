import logging
import threading
import time

from groundstation.utils import sendstate, add_logline


class SSHPing:

    def __init__(self, client):
        self.client = client
        self.last_upstime_seconds = 0

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
            # This file contains two numbers: the uptime of the system (seconds),
            # and the amount of time spent in idle process (seconds).
            (exitcode, proc_uptime) = self.client.run_command("cat /proc/uptime")
            if exitcode != 0:
                raise Exception("uptime exit code was not 0")
            connected = True
            uptime_seconds = float(proc_uptime.strip().split(" ")[0])
            if self.last_upstime_seconds > uptime_seconds:
                add_logline(time.time(), "Luke powercycled", "sshping")
            self.last_upstime_seconds = uptime_seconds
            uptimestr = time.strftime("%Hh %Mm %Ss", time.gmtime(uptime_seconds))
        except Exception, e:
            uptimestr = str(e)
            connected = False

        sendstate({'ssh': {
            'connected': connected,
            'uptime': uptimestr,
            'lastping': time.time()
        }})
