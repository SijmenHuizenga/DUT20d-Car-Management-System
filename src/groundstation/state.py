from utils.dict_merge import dict_merge
from database import database as db


class StateManager:
    def __init__(self):
        self.state = {
            'pinger': {
                'computebox': {
                    'timestamp': 0.0,
                    'success': False,
                }
            },
            'rosnode': {
                'up': False,
            },
            'logbook': [],
            'topics': [],
            'recording': {
                'is_recording': False,
                'bagname': "sysid_corner1_vioenabled",
                'bagfilename': "sysid_corner1_vioenabled_20190804172203.bag.active",
                'recordingduration': "03:42",
                'selected_topics': [
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

    def set_sio(self, sio):
        self.sio = sio


statemanager = StateManager()
