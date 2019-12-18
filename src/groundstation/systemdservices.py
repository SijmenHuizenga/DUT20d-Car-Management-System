import threading
import time


services = [
    "inspection_mission.service",
    "rosrecord.service",
    "mavros.service",
    "manualjoycontrol.service"
]


def statusnr_to_string(statusnr):
    #  0: program is running or service is OK
    #  1: program is dead and /run PID file exists
    #  2: program is dead and /run/lock lock file exists
    #  3: program is not running
    #  4: program or service status is unknown
    if statusnr == 0:
        return "running"
    if 1 <= statusnr <= 3:
        return "stopped"
    if statusnr == 4:
        return "error"
    raise Exception("Unkown status code " + statusnr)


class SystemdServices:
    def __init__(self, ssh, state):
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
            time.sleep(5)

    def retreive_services(self):
        for service in services:
            self.retreive_service(service)

    def retreive_service(self, service):
        try:
            statusnr, output = self.ssh.run_command("systemctl is-enabled " + service)
            enabled = statusnr == 0 and output.strip() != "static"
            statusnr, statustext = self.ssh.run_command("systemctl status " + service)
            status = statusnr_to_string(statusnr)

        except Exception, e:
            status = "error"
            statustext = str(e)
            enabled = "unkown"
        self.state.update({
            'systemdservices': {
                service: {
                    'status': status,
                    'statustext': statustext,
                    'enabled': enabled,
                    'lastupdate': time.time()
                }
            }
        })
