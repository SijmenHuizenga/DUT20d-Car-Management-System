import threading
import time

import systemdservices
from .sshclient import SSHClient
from .state import State, SystemdServiceRunning


class RosRecorder:
    def __init__(self,
                 state,  # type: State
                 ssh  # type: SSHClient
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
                self.updatestate()
            except Exception, e:
                print("[pinger] Someting unexpected happend updating recording state: %s" % str(e))
            time.sleep(5)

    def updatestate(self):
        (filename, selectedtopics) = self.read_configfile()

        self.state.recording.is_recording = self.is_recording()
        self.state.recording.lastrefresh = time.time()
        self.state.recording.filename = filename
        self.state.recording.bagfilename = "fakedata.bag.active"
        self.state.recording.recordingduration = "00:00"
        self.state.recording.selected_topics = selectedtopics
        self.state.emit_state()

    def is_recording(self):
        try:
            statusnr, statustext = self.ssh.run_command("sudo systemctl status rosrecord.service")
            return systemdservices.statusnr_to_running(statusnr) == SystemdServiceRunning.RUNNING
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

    def set_topic(self, topicname, selected):
        newtopics = self.state.recording.selected_topics[:]
        if topicname in newtopics:
            newtopics.remove(topicname)
        if selected:
            newtopics.append(topicname)

        self.save_configuration(newtopics, self.state.recording.filename)
        self.updatestate()

    def set_filename(self, newfilename):
        self.save_configuration(self.state.recording.selected_topics, newfilename)
        self.updatestate()

    def save_configuration(self, selectedtopics, filename):
        topicstr = ' '.join(selectedtopics)
        statusnr, output = self.ssh.run_command('echo \"RECORD_TOPICS=' + topicstr + '\n' +
                                                'RECORD_FILENAME=' + filename + '\n\" > /var/rosrecord.env')
        if statusnr != 0:
            raise Exception("Updating config file failed with code " + str(statusnr) + ": " + output)
        return

    def recording_start(self):
        statusnr, statustext = self.ssh.run_command("sudo systemctl start rosrecord.service")
        if statusnr != 0:
            raise Exception("systemctl start rosrecord.service exited with error code " +
                            str(statusnr) + ": " + statustext)
        self.updatestate()

    def recording_stop(self):
        statusnr, statustext = self.ssh.run_command("sudo systemctl stop rosrecord.service")
        if statusnr != 0:
            raise Exception("systemctl stop rosrecord.service exited with error code " +
                            str(statusnr) + ": " + statustext)
        self.updatestate()
