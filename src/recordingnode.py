#!/usr/bin/env python
import os
import datetime
import subprocess
import rospy
import time
from cms.msg import RecordingStatus, RecordingConfig, BagTopicInfo
from cms.srv import RecordingUpdateConfig, RecordingUpdateConfigResponse, RecordingGetBagInfo, RecordingGetBagInfoResponse
from rosbag.bag import Bag, ROSBagException


class RosbagRecordNode:

    def __init__(self):
        rospy.init_node('cms_recording', anonymous=True)

        self.recording_status_pub = rospy.Publisher('/cms/recording/status', RecordingStatus, queue_size=1)
        self.recording_config_pub = rospy.Publisher('/cms/recording/config', RecordingConfig, queue_size=1)

        rospy.Timer(rospy.Duration(3), self.publish_recordingconfig)
        rospy.Timer(rospy.Duration(1), self.publish_recordingstatus)

        rospy.Service('/cms/recording/updateconfig', RecordingUpdateConfig, self.update_recording_config)
        rospy.Service('/cms/recording/baginfo', RecordingGetBagInfo, self.rosbaginfo)

        rospy.spin()

    def read_recordconfigfile(self):
        with open('/var/rosrecord.env', 'r') as configfile:
            configtext = configfile.read()
            config = RecordingConfig()
            config.topics = []
            config.filename = "Could not find filename in /var/rosrecord.env file"
            for line in configtext.strip().split("\n"):
                if line.strip().startswith("RECORD_TOPICS="):
                    config.topics = line[len("RECORD_TOPICS="):].split(" ")
                if line.strip().startswith("RECORD_FILENAME="):
                    config.filename = line[len("RECORD_FILENAME="):]
            return config

    def publish_recordingconfig(self, event):
        try:
            self.recording_config_pub.publish(self.read_recordconfigfile())
        except Exception, e:
            rospy.logerr("reading /var/rosrecord.env failed with exception: " + str(e))

    def publish_recordingstatus(self, event):
        try:
            statustext = subprocess.check_output(['systemctl', 'status', 'rosrecord.service'])

            # Searching for a line like this:
            # rosbag record -O /storage/bags/2020-01-22_15-04-51_VIO_data /camera/image_raw /mavros/imu/data_raw
            q = 'rosbag record -O '
            sindex = statustext.index(q)
            parts = statustext[sindex+len(q):statustext.index('\n', sindex)].strip().split(' ')
            statusmsg = RecordingStatus()
            statusmsg.recordingactive = True
            statusmsg.filename = parts[0] + '.bag.active'
            statusmsg.topics = parts[1:]
            statusmsg.filesize = int(os.path.getsize(statusmsg.filename) / 1e+6)

            activatedtext = subprocess.check_output(['systemctl', 'show', 'rosrecord', '--property=ActiveEnterTimestamp'])
            # Expect an output like:
            # ActiveEnterTimestamp=Wed 2020-01-22 15:04:51 CET
            timestamp = activatedtext[len('ActiveEnterTimestamp='):].strip()
            statusmsg.duration = int((datetime.datetime.now() - datetime.datetime.strptime(timestamp, "%a %Y-%m-%d %H:%M:%S %Z")).total_seconds())

            self.recording_status_pub.publish(statusmsg)
        except subprocess.CalledProcessError:
            statusmsg = RecordingStatus()
            statusmsg.recordingactive = False
            self.recording_status_pub.publish(statusmsg)
        except ValueError, e:
            # Raised when string.index() fails
            rospy.logerr("Failed parsing systemctl output: " + str(e))
        except os.error, e:
            # Raised when os.path.getsize fails
            rospy.logerr("Could not get filesize: " + str(e))

    def update_recording_config(self, req):
        start = time.time()

        with open("/var/rosrecord.env", "w") as f:
            f.write('RECORD_TOPICS=' + (' '.join(req.config.topics)) + '\n')
            f.write('RECORD_FILENAME=' + req.config.filename + '\n')
        response = RecordingUpdateConfigResponse()
        response.config = self.read_recordconfigfile()
        return response

    def rosbaginfo(self, req):
        response = RecordingGetBagInfoResponse()
        response.filesize = int(os.path.getsize(req.filename) / 1e+6)

        bag = Bag(req.filename, 'r')
        response.filename = req.filename
        response.starttime = bag.get_start_time()
        response.endtime = bag.get_end_time()
        response.topics = []

        topicmap = bag.get_type_and_topic_info()[1]
        for topic in topicmap.keys():
            topicinfo = BagTopicInfo()
            topicinfo.topicname = topic
            topicinfo.messagecount = topicmap[topic].message_count
            response.topics.append(topicinfo)
        return response


if __name__ == '__main__':
    RosbagRecordNode()
