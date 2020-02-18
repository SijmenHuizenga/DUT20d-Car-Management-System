import threading
import time

from groundstation.models import SystemdServiceRunning, SystemdServiceEnabled, SystemdService
from groundstation.utils import sendstate

services = [
    "roscore.service",
    "rosrecord.service",
    "cameras.service",
    "infrastructure.service",
    "lidar.service",
    "missioncontrol.service",
    "se.service",
    "statistics.service",
    "vehicleinterface.service",
    "pixhawk.service",
    "mission_acceleration.service",
    "mission_inspection.service",
    "mission_joystick.service",
    "mission_skidpad.service",
    "mission_trackdrive.service",
]


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
                print("[systemdservices] Someting unexpected happend while retreiving services: " + str(e))
            time.sleep(1)

    def retreive_services(self):
        services_str = " ".join(services)

        try:
            statusnr_enabled, output_enabled = self.ssh.run_command("systemctl is-enabled " + services_str)
            statusnr_active, output_active = self.ssh.run_command("systemctl is-active " + services_str)
        except Exception, e:
            out = []
            for s in services:
                out.append(SystemdService(name=s, running=SystemdServiceRunning.ERROR, statustext=str(e),
                                          enabled=SystemdServiceEnabled.ERROR, lastupdate=time.time()))
            sendstate({'systemdservices': out})
            return

        services_active = output_active.split("\n")
        services_enabled = output_enabled.split("\n")

        out = []
        for i in range(len(services)):
            out.append(SystemdService(name=services[i], lastupdate=time.time(),
                                      running=SystemdServiceRunning.fromstring(services_active[i]),
                                      enabled=SystemdServiceEnabled.fromstring(services_enabled[i]),
                                      statustext="active: " + services_active[i] + "\n" + "enabled: " + services_enabled[i]))

        sendstate({'systemdservices': out})
