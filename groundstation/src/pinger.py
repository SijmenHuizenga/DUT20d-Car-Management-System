import threading
import os
import time
from state import statemanager as state

computeboxHostname = "google.com"


def initialize():
    thread = threading.Thread(target=ping_computebox_forever)
    thread.daemon = True
    thread.start()


def ping_computebox_forever():
    while 1:
        try:
            ping_computebox()
        except Exception, e:
            print("[pinger] Someting unexpected happend while pinging the computebox: " + str(e))
            pass
        time.sleep(1)


def ping_computebox():
    global computeboxHostname
    response = os.system("ping -c 1 -w 1 -W 1 " + computeboxHostname + " > /dev/null 2>&1")
    state.did_ping("computebox", response == 0)
