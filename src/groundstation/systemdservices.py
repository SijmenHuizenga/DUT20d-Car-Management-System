import threading
import time

from .sshclient import SSHClient
from .state import State, SystemdService, SystemdServiceRunning, SystemdServiceEnabled

services = [
    "inspection_mission.service",
    "rosrecord.service",
    "mavros.service",
    "manualjoycontrol.service"
]


def statusnr_to_running(statusnr):
    #  0: program is running or service is OK
    #  1: program is dead and /run PID file exists
    #  2: program is dead and /run/lock lock file exists
    #  3: program is not running
    #  4: program or service status is unknown
    if statusnr == 0:
        return SystemdServiceRunning.RUNNING
    if 1 <= statusnr <= 3:
        return SystemdServiceRunning.STOPPED
    if statusnr == 4:
        raise Exception("program or service status is unknown")
    raise Exception("Unkown status code " + statusnr)


class SystemdServices:
    def __init__(self,
                 ssh,  # type: SSHClient
                 state  # type: State
                 ):
        self.ssh = ssh
        self.state = state

    def start(self):
        thread = threading.Thread(target=self.forever)
        thread.daemon = True
        thread.start()

    def forever(self):
        while 1:
            try:
                self.retreive_services()
            except Exception, e:
                print("[pinger] Someting unexpected happend while retreiving services: " + str(e))
            time.sleep(3)

    def retreive_services(self):
        self.state.systemdservices = map(self.retreive_service, services)

    def retreive_service(self, service):
        try:
            statusnr, output = self.ssh.run_command("systemctl is-enabled " + service)
            enabled = SystemdServiceEnabled.ENABLED \
                if statusnr == 0 and output.strip() != "static" \
                else SystemdServiceEnabled.DISABLED
            statusnr, statustext = self.ssh.run_command("systemctl status " + service)
            running = statusnr_to_running(statusnr)

        except Exception, e:
            running = SystemdServiceRunning.ERROR
            statustext = str(e)
            enabled = SystemdServiceEnabled.ERROR

        return SystemdService(name=service, running=running, statustext=statustext, enabled=enabled, lastupdate=time.time())
