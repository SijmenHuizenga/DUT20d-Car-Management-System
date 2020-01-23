import time
import rospy
from cms.msg import RecordingConfig
from cms.srv import RecordingUpdateConfigRequest

from groundstation.logbook import Logbook
from .sshclient import SSHClient
from .state import State


class RosRecorder:
    def __init__(self,
                 state,  # type: State
                 ssh,  # type: SSHClient
                 logbook  # type: Logbook
                 ):
        self.ssh = ssh
        self.state = state
        self.logbook = logbook
        self.update_recordingconfig_service = None
        self.rosbaginfo_service = None
        self.statuscallback_first = True

    def set_service_proxies(self, update_recordingconfig_service, rosbaginfo_service):
        self.update_recordingconfig_service = update_recordingconfig_service
        self.rosbaginfo_service = rosbaginfo_service

    def config_callback(self, config):
        self.state.recording.config_topics = config.topics
        self.state.recording.config_filename = config.filename
        self.state.recording.lastrefresh_config = time.time()
        self.state.emit_state()

    def status_callback(self, status):
        if not self.statuscallback_first:
            if not self.state.recording.is_recording and status.recordingactive:
                self.logbook.add_line(time.time(), "Recording started, filename: %s" % status.filename, "recorder")
            if self.state.recording.is_recording and not status.recordingactive:
                self.logbook.add_line(time.time(), "Recording stopped after %s seconds." % self.state.recording.recording_duration, "recorder")

        self.state.recording.is_recording = status.recordingactive
        self.state.recording.recording_file = status.filename
        self.state.recording.recording_duration = status.duration
        self.state.recording.recording_filesize = status.filesize
        self.state.recording.recording_topics = status.topics
        self.state.recording.lastrefresh_recording = time.time()
        self.state.emit_state()
        self.statuscallback_first = False

    def set_topic(self, topicname, selected):
        newtopics = self.state.recording.config_topics[:]
        if topicname in newtopics:
            newtopics.remove(topicname)
        if selected:
            newtopics.append(topicname)

        self.save_configuration(newtopics, self.state.recording.config_filename)

    def set_filename(self, newfilename):
        self.save_configuration(self.state.recording.config_topics, newfilename)

    def save_configuration(self, selectedtopics, filename):
        config = RecordingConfig()
        config.filename = filename
        config.topics = selectedtopics
        try:
            response = self.update_recordingconfig_service(RecordingUpdateConfigRequest(config))
            config = response.config
        except rospy.ServiceException, e:
            raise e

        self.config_callback(config)

    def recording_start(self):
        statusnr, statustext = self.ssh.run_command("sudo systemctl start rosrecord.service")
        if statusnr != 0:
            raise Exception("systemctl start rosrecord.service exited with error code " +
                            str(statusnr) + ": " + statustext)

    def recording_stop(self):
        statusnr, statustext = self.ssh.run_command("sudo systemctl stop rosrecord.service")
        if statusnr != 0:
            raise Exception("systemctl stop rosrecord.service exited with error code " +
                            str(statusnr) + ": " + statustext)
