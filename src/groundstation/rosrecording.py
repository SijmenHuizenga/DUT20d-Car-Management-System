import threading
import time
import systemdservices


class RosRecorder:
    def __init__(self, state, ssh):
        self.ssh = ssh
        self.state = state

        self.start()

    def start(self):
        thread = threading.Thread(target=self.forever)
        thread.daemon = True
        thread.start()

    def forever(self):
        while 1:
            try:
                self.updatestate()
            except Exception, e:
                print("[pinger] Someting unexpected happend updating recording state: %s" % str(e))
            time.sleep(5)

    def updatestate(self):
        (filename, selectedtopics) = self.read_configfile()
        self.state.update({
            'recording': {
                'is_recording': self.is_recording(),
                'filename': filename,
                'bagfilename': "fakedata.bag.active",
                'recordingduration': "00:00",
                'selected_topics': selectedtopics,
            }
        })

    def is_recording(self):
        try:
            statusnr, statustext = self.ssh.run_command("systemctl status rosrecording.service")
            return systemdservices.statusnr_to_string(statusnr) == "running"
        except Exception, e:
            return False

    def read_configfile(self):
        try:
            statusnr, text = self.ssh.run_command("cat /var/rosrecord.env")
            if statusnr != 0:
                raise Exception("'cat /var/rosrecord.env' failed with status code " + str(statusnr) + ": " + text)

            topics = []
            filename = "Could not find filename in /var/rosrecord.env file"
            for line in text.strip().split("\n"):
                if line.strip().startswith("RECORD_TOPICS="):
                    topics = line[len("RECORD_TOPICS="):].split(" ")
                if line.strip().startswith("RECORD_FILENAME="):
                    filename = line[len("RECORD_FILENAME="):]
            return filename, topics
        except Exception, e:
            return "error", str(e)
