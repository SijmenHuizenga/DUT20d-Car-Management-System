from utils.dict_merge import dict_merge


class StateManager:
    def __init__(self, db):
        self.db = db
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
            'systemdservices': {

            },
            'recording': {
                'is_recording': False,
                'filename': "unset",
                'bagfilename': "fakedata.bag.active",
                'recordingduration': "00:00",
                'selected_topics': [],
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
        self.state['logbook'] = self.db.select_all("SELECT rowid, * FROM logbook", {})

        alltimetopics = self.db.select_all("SELECT name FROM topics GROUP BY name", {})
        for topic in alltimetopics:
            self.state['topics'][topic['name']] = {
                'lastseen': -1,
                'type': 'unkown',
            }

    def set_sio(self, sio):
        self.sio = sio
