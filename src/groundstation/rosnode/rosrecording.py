import time

import rospy
from cms.msg import RecordingConfig
from cms.srv import RecordingUpdateConfigRequest

from groundstation.models import Recording
from groundstation.utils import add_logline, sendstate


class RosRecorder:

    def __init__(self):
        self.update_recordingconfig_service = None
        self.rosbaginfo_service = None
        self.statuscallback_first = True
        self.recording = Recording()

    def set_service_proxies(self, update_recordingconfig_service, rosbaginfo_service):
        self.update_recordingconfig_service = update_recordingconfig_service
        self.rosbaginfo_service = rosbaginfo_service

    def config_callback(self, config):
        self.recording.config_topics = config.topics
        self.recording.config_filename = config.filename
        self.recording.lastrefresh_config = time.time()
        sendstate({'recording': self.recording})

    def status_callback(self, status):
        if not self.statuscallback_first:
            if not self.recording.is_recording and status.recordingactive:
                add_logline(time.time(), "Recording started, filename: %s" % status.filename, "recorder")
            if self.recording.is_recording and not status.recordingactive:
                add_logline(time.time(), "Recording stopped after %s seconds." % self.recording.recording_duration, "recorder")

        self.recording.is_recording = status.recordingactive
        self.recording.recording_file = status.filename
        self.recording.recording_duration = status.duration
        self.recording.recording_filesize = status.filesize
        self.recording.recording_topics = status.topics
        self.recording.lastrefresh_recording = time.time()
        self.statuscallback_first = False
        sendstate({'recording': self.recording})

    def set_topics(self, topicnames, selected):
        newtopics = self.recording.config_topics[:]
        for topicname in topicnames:
            if topicname in newtopics:
                newtopics.remove(topicname)
            if selected:
                newtopics.append(topicname)

        self.save_configuration(newtopics, self.recording.config_filename)

    def set_filename(self, newfilename):
        self.save_configuration(self.recording.config_topics, newfilename)

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