import os
import threading
import time


class SystemdServices:
    def __init__(self, ssh):
        self.ssh = ssh

    def start(self):
        thread = threading.Thread(target=self.forever)
        thread.daemon = True
        thread.start()

    def forever(self):
        while 1:
            try:
                self.retreive_services()
            except Exception, e:
                print("[pinger] Someting unexpected happend while pinging retreiving services: " + str(e))
            time.sleep(5)

    def retreive_services(self):
        ssh.
