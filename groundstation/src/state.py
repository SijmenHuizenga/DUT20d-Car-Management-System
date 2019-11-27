import time
import sqlite3
import threading


class StateManager:
    def __init__(self):
        self.database = sqlite3.connect('development.db', check_same_thread=False)
        self.databaselock = threading.Lock()
        self.create_database()

        self.state = {
            'pinger': {
                'computebox': {
                    'timestamp': 0,
                    'success': False,
                }
            },
            'rosnode': {
                'up': False,
            },
            'logbook': [
                {
                    'creation_timestamp': 0,
                    'text': "Hello World",
                    'source': "user",
                }
            ]
        }

    def dbquery(self, query, args):
        self.databaselock.acquire()
        with self.database:
            self.database.execute(query, args)
        self.databaselock.release()

    def create_database(self):
        self.databaselock.acquire()
        with self.database:
            self.database.execute('CREATE TABLE IF NOT EXISTS pings (timestamp FLOAT, device VARCHAR(10), success BOOLEAN)')
            self.database.execute('CREATE TABLE IF NOT EXISTS rosnodehealth (timestamp FLOAT, up BOOLEAN)')
        self.databaselock.release()

    def did_ping(self, device, success):
        now = time.time()
        self.state['pinger'][device]['timestamp'] = now
        self.state['pinger'][device]['success'] = success
        self.dbquery('''INSERT INTO pings VALUES (:now, :device, :success)''',
                     {"now": now, "device": device, "success": success})

    def set_rosnode_health(self, up):
        self.state['rosnode']['up'] = up
        self.dbquery('''INSERT INTO rosnodehealth VALUES (:now, :up)''',
                     {"now": time.time(), "up": up})


statemanager = StateManager()
