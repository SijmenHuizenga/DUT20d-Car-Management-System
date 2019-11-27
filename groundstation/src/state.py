import time

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


def did_computebox_ping():
    global state
    state['pinger']['computebox']['last_run'] = time.time()


def did_successful_computebox_ping():
    global state
    state['pinger']['computebox']['last_success'] = time.time()


def set_rosnode_health(is_up):
    global state
    state['rosnode']['up'] = is_up