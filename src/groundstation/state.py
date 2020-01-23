from typing import List, Optional

import jsonpickle
import time
import threading

class Ping:
    def __init__(self):
        # The unix timestamp in seconds when the last network ping to the computer was executed.
        self.timestamp = 0.0  # type: float

        # Whether or not the last ping at `timestamp` succeeded.
        self.success = False  # type: bool


class SSH:
    def __init__(self):
        # Whether or not the ssh connection is usable.
        self.connected = False  # type: bool

        # The last time a successfull ping over the ssh connection was done
        self.lastping = 0.0  # type: float

        # The single-line result from the `uptime` command.
        # The `uptime` command is used as ssh ping so this field will show anny errors in case `connected` is False
        self.uptime = "unkown"  # type: str


class Rosnode:
    def __init__(self):
        # Whether or not the groundstation ros node is connected to the ros master
        self.up = False  # type: bool


class LogbookLine:
    def __init__(self, timestamp, text, source, rowid=None):
        # The timestamp on which the line resides.
        # If the line is updated the old value is replaced by the new.
        self.timestamp = timestamp  # type: float

        # The multiline and markdown supported text.
        self.text = text  # type: str

        # Who made this logline? This field keeps track of who issued the line.
        self.source = source  # type: str

        # The id of the row in the database. Optional.
        self.rowid = rowid  # type: Optional[int]


# YES I KNOW ENUMS EXIST BUT I JUST SPENT 90 MINUTES TRYING
# TO GET THEM TO WORK AND I FAILD SO NOW WE USE STRINGS AAH
# IF YOU WANT ENUMS GO AHEAD FIX IT PLZZ thank you
class SystemdServiceRunning:
    def __init__(self):
        pass

    # The service is running
    RUNNING = "running"

    # The service is not running, can be either stopped or crashed.
    STOPPED = "stopped"

    # Something went wrong while retreiving the status of the service.
    ERROR = "error"


class SystemdServiceEnabled:
    def __init__(self):
        pass

    # The service is enabled and will start on boot
    ENABLED = "enabled"

    # The service is disabled and won't start on boot
    DISABLED = "disabled"

    # Something went wrong while retreiving the enabled status of the service.
    ERROR = "error"


class SystemdService:
    def __init__(self, name, running, statustext, enabled, lastupdate):
        # The name of the service, including .service suffix.
        self.name = name  # type: str
        # Is the service running?
        self.running = running  # type: SystemdServiceRunning
        # The output of `systemctl status <servicename>`
        self.statustext = statustext  # type: str
        # Is the service enabled?
        self.enabled = enabled  # type: SystemdServiceEnabled
        # When was this information last updated?
        self.lastupdate = lastupdate  # type: float


class Recording:
    def __init__(self):
        # Wheather or not we are currently recording.
        self.is_recording = False  # type: bool

        # The topics selected for recording.
        self.config_topics = []  # type: List[str]

        # The filename that is configued for upcomming recordings.
        self.config_filename = "fakedata"  # type: str

        # The name of the rosbag that is currently recording
        self.recording_file = "/data/bags/fakedata.bag.active"  # type: str

        # The duration of the duration. Only valid if is_recording==True. Seconds
        self.recording_duration = 60  # type: int

        # The size of the recording_file in megabyte
        self.recording_filesize = 1500  # type: int

        # The topics that are being recorded.
        self.recording_topics = []  # type: List[str]

        # When was the recording_* information last refreshed?
        self.lastrefresh_recording = 0  # type: float

        # When was the config_* information last refreshed?
        self.lastrefresh_config = 0  # type: float


class Topic:
    def __init__(self, name, lastseen, statistics):
        # The name of the topic, including / prefix
        self.name = name  # type: str

        # A timestamp of when this topic was last seen.
        # This parameter is updated by all places that receive metadata about topics.
        # This does not tell anything about activeness: a topic might be lastseen seconds ago
        # but not have any publishers / subscribers for ages. However, if this timestamp is
        # set to a long time ago you can be sure this topic is not there.
        self.lastseen = lastseen  # type: float

        self.statistics = statistics  # type: Optional[TopicStatistic]


class TopicType:
    def __init__(self, topicname, lastseen, messagetype):
        # The name of the topic, including / prefix
        self.topicname = topicname  # type: str

        # The name of the ros message
        self.messagetype = messagetype

        # The timestamp when the information about this topic was last updated
        self.lastseen = lastseen  # type: float


class Node:
    def __init__(self, name, lastseen):
        # The name of the node, including / prefix
        self.name = name  # type: str

        # A timestamp of when this topic was last seen.
        # This parameter is updated by all places that receive metadata about topics.
        # This does not tell anything about activeness: a topic might be lastseen seconds ago
        # but not have any publishers / subscribers for ages. However, if this timestamp is
        # set to a long time ago you can be sure this topic is not there.
        self.lastseen = lastseen  # type: float


class TopicPublication:
    def __init__(self, nodename, topicname, lastseen):
        # The name of the node, including / prefix
        self.nodename = nodename  # type: str

        # The name of the topic, including / prefix
        self.topicname = topicname  # type: str

        # The timestamp when the information about this node was last updated
        self.lastseen = lastseen  # type: float


class TopicSubscription:
    def __init__(self, nodename, topicname, lastseen):
        # The name of the node, including / prefix
        self.nodename = nodename  # type: str

        # The name of the topic, including / prefix
        self.topicname = topicname  # type: str

        # The timestamp when the information about this node was last updated
        self.lastseen = lastseen  # type: float


class TopicStatistic:
    def __init__(self, traffic, lastseen):
        # The amount of messages per second since the last statistic.
        self.traffic = traffic

        # The timestamp when the information about this node was last updated
        self.lastseen = lastseen  # type: float


class State:
    ping = Ping()
    ssh = SSH()
    rosnode = Rosnode()
    recording = Recording()
    logbook = []  # type: List[LogbookLine]
    systemdservices = []  # type: List[SystemdService]
    topics = []  # type: List[Topic]
    nodes = []  # type: List[Node]
    topictypes = []  # type: List[TopicType]
    subscriptions = []  # type: List[TopicSubscription]
    publications = []  # type: List[TopicPublication]

    lastping = 0
    somethingtoemit = True

    def __init__(self, db):
        self.db = db
        self.populate_memory_state()
        self.socketio = None

    def start(self):
        thread = threading.Thread(target=self.emit_state_thread)
        thread.daemon = True
        thread.start()

    def emit_state_thread(self):
        while 1:
            time.sleep(0.1)
            if self.socketio is None:
                continue
            timesincelastemit = time.time() - self.lastping
            if timesincelastemit < 0.3:
                # do not emit state if last emitted state was less then 0.3 s ago
                continue
            if timesincelastemit > 1 or self.somethingtoemit:
                self.socketio.emit('state', self.to_json(), broadcast=True)
                self.lastping = time.time()
                self.somethingtoemit = False


    def emit_state(self):
        self.somethingtoemit = True

    def populate_memory_state(self):
        self.logbook = map(self.logline_fromdict, self.db.select_all("SELECT rowid, * FROM logbook", {}))

    def logline_fromdict(self, d):
        return LogbookLine(d['timestamp'], d['text'], d['source'], d['rowid'])

    def set_socketio(self, sio):
        self.socketio = sio

    def to_json(self):
        return jsonpickle.encode(self, unpicklable=False, warn=True)

    # Used by jsonpickle to get the available fields
    def __getstate__(self):
        return {
            'ping': self.ping,
            'ssh': self.ssh,
            'rosnode': self.rosnode,
            'recording': self.recording,
            'logbook': self.logbook,
            'topics': self.topics,
            'nodes': self.nodes,
            'systemdservices': self.systemdservices,
            'topictypes': self.topictypes,
            'subscriptions': self.subscriptions,
            'publications': self.publications,
        }
