import jsonpickle
from typing import List, Optional


class Ping:
    def __init__(self):
        pass

    # The unix timestamp in seconds when the last network ping to the computer was executed.
    timestamp = 0.0  # type: float

    # Whether or not the last ping at `timestamp` succeeded.
    success = False  # type: bool


class SSH:
    def __init__(self):
        pass

    # Whether or not the ssh connection is usable.
    connected = False  # type: bool

    # The last time a successfull ping over the ssh connection was done
    lastping = 0.0  # type: float

    # The single-line result from the `uptime` command.
    # The `uptime` command is used as ssh ping so this field will show anny errors in case `connected` is False
    uptime = "unkown"  # type: str


class Rosnode:
    def __init__(self):
        pass

    # Whether or not the groundstation ros node is connected to the ros master
    up = False  # type: bool


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
        pass

    # Wheather or not we are currently recording.
    is_recording = False  # type: bool

    # The filename that is configued for upcomming recordings.
    filename = "unset"  # type: str

    # The name of the rosbag that is currently recording
    bagfilename = "fakedata.bag.active"  # type: str

    # The duration of the duration. Only valid if is_recording==True
    recordingduration = "00:00"  # type: str

    # The topics selected for recording.
    selected_topics = []  # type: List[str]

    # When was this information last refreshed?
    lastrefresh = 0  # type: float


class State:
    ping = Ping()
    ssh = SSH()
    rosnode = Rosnode()
    recording = Recording()
    logbook = []  # type: List[LogbookLine]
    topics = {}
    nodes = {}
    systemdservices = []  # type: List[SystemdService]

    def __init__(self, db):
        self.db = db
        self.populate_memory_state()
        self.socketio = None

    def emit_state(self):
        assert self.socketio is not None, "sio was not set"
        self.socketio.emit('state', self.to_json(), broadcast=True)

    def populate_memory_state(self):
        self.logbook = self.db.select_all("SELECT rowid, * FROM logbook", {})

        alltimetopics = self.db.select_all("SELECT name FROM topics GROUP BY name", {})
        for topic in alltimetopics:
            self.topics[topic['name']] = {
                'lastseen': -1,
                'type': 'unkown',
            }

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
        }
