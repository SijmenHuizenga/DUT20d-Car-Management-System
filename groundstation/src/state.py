import time

from database import Database


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
            'logbook': []
        }
        self.db = Database()
        self.populate_memory_state()
        self.sio = None

    def emit_state(self):
        self.sio.emit('state', self.state, broadcast=True)

    def populate_memory_state(self):
        self.state['logbook'] = self.db.select_all("SELECT rowid, * FROM logbook", {})

    def did_ping(self, device, success):
        now = time.time()
        self.db.insert('INSERT INTO pings VALUES (:now, :device, :success)',
                       {'now': now, 'device': device, 'success': success})
        self.state['pinger'][device]['timestamp'] = now
        self.state['pinger'][device]['success'] = success
        self.emit_state()

    def set_rosnode_health(self, up):
        self.db.insert('INSERT INTO rosnodehealth VALUES (:now, :up)',
                       {'now': time.time(), 'up': up})
        self.state['rosnode']['up'] = up
        self.emit_state()

    def add_logbook_line(self, timestamp, text, source):
        line = {'timestamp': timestamp, 'text': text, 'source': source}
        line['rowid'] = self.db.insert('INSERT INTO logbook VALUES (:timestamp, :text, :source)', line)
        self.state['logbook'].append(line)
        self.emit_state()

    def set_sio(self, sio):
        self.sio = sio


statemanager = StateManager()
