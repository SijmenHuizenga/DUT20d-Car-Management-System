import time
import sqlite3
import threading
database = sqlite3.connect('development.db', check_same_thread=False)
databaselock = threading.Lock()

state = {
    'pinger': {
        'computebox': {
            'last_success': 0,
            'last_run': 0,
        }
    },
    'rosnode': {
        'up': False,
    }
}


def create_database():
    global database
    global databaselock
    databaselock.acquire()
    with database:
        database.execute('CREATE TABLE IF NOT EXISTS pings (timestamp FLOAT, device VARCHAR(10), success BOOLEAN)')
    databaselock.release()


def did_ping(device, success):
    global state
    global database
    now = time.time()
    state['pinger'][device]['timestamp'] = now
    state['pinger'][device]['success'] = success
    if success:
        state['pinger'][device]['last_success'] = now
    databaselock.acquire()
    with database:
        database.execute('''INSERT INTO pings VALUES (:now, :device, :success)''',
                         {"now": now, "device": device, "success": success})
    databaselock.release()


def set_rosnode_health(is_up):
    global state
    state['rosnode']['up'] = is_up


create_database()
