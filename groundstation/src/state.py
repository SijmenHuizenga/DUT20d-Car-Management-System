import time
import sqlite3
import threading
database = sqlite3.connect('development.db', check_same_thread=False)
databaselock = threading.Lock()

state = {
    'pinger': {
        'computebox': {
            'timestamp': 0,
            'success': False,
        }
    },
    'rosnode': {
        'up': False,
    }
}


def dbquery(query, args):
    databaselock.acquire()
    with database:
        database.execute(query, args)
    databaselock.release()


def create_database():
    global database
    global databaselock
    databaselock.acquire()
    with database:
        database.execute('CREATE TABLE IF NOT EXISTS pings (timestamp FLOAT, device VARCHAR(10), success BOOLEAN)')
        database.execute('CREATE TABLE IF NOT EXISTS rosnodehealth (timestamp FLOAT, up BOOLEAN)')
    databaselock.release()


def did_ping(device, success):
    global state
    global database
    now = time.time()
    state['pinger'][device]['timestamp'] = now
    state['pinger'][device]['success'] = success
    dbquery('''INSERT INTO pings VALUES (:now, :device, :success)''',
            {"now": now, "device": device, "success": success})


def set_rosnode_health(up):
    global state
    state['rosnode']['up'] = up
    dbquery('''INSERT INTO rosnodehealth VALUES (:now, :up)''',
            {"now": time.time(), "up": up})


create_database()
