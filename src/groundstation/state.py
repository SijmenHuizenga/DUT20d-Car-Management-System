from utils.dict_merge import dict_merge
from database import database as db


class StateManager:
    def __init__(self):
        self.state = {
            'ping': {
                # 'timestamp': 0.0,
                # 'success': False,
            },
            'ssh': {
                # 'connected': False,
                # 'uptime': 'up 11 hours, 52 minutes',
                # 'lastping': 123456
            },
            'rosnode': {
                # 'up': False,
            },
            'logbook': [],
            'topics': {
                # '/rosout': {'lastseen': 123456,
                #             'type': 'rosgraph_msgs/Log'}
            },
            'nodes': {
                # '/cms': {'lastseen': 123456,
                #          'subscriptions': ['/rosout', '/statistics'],
                #          'publications': ['/example']}
            },
            'recording': {
                'is_recording': False,
                'bagname': "sysid_corner1_vioenabled",
                'bagfilename': "sysid_corner1_vioenabled_20190804172203.bag.active",
                'recordingduration': "03:42",
                'selected_topics': [
                    "/rosout"
                    "/planning_ReferencePath",
                    "/visualization_markers/world_state",
                    "/mavros/local_position/velocity_local",
                    "/world_state",
                    "/mavros/local_position/pose",
                ],
            }
        }

        self.populate_memory_state()
        self.sio = None

    def update(self, changeset):
        dict_merge(self.state, changeset)
        self.emit_state()

    def emit_state(self):
        self.sio.emit('state', self.state, broadcast=True)

    def populate_memory_state(self):
        self.state['logbook'] = db.select_all("SELECT rowid, * FROM logbook", {})

        alltimetopics = db.select_all("SELECT name FROM topics GROUP BY name", {})
        for topic in alltimetopics:
            self.state['topics'][topic['name']] = {
                'lastseen': -1,
                'type': 'unkown',
            }

    def set_sio(self, sio):
        self.sio = sio


statemanager = StateManager()
