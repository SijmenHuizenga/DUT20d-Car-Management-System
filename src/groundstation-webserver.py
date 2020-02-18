#!/usr/bin/env python3

# To test this script from the command line you can use the following command
# > curl --data '{"text1": "1"}' -X POST http://localhost:1097/state --header "Content-Type:application/json"


# Initializes the eventlet async webserver.
# This is required for multithreaded message emitting over websocket.
# Needs to be the first import in the whole program. Read more about it here:
#  https://eventlet.net/doc/patching.html
import eventlet

from groundstation.utils import update

eventlet.monkey_patch()

import logging
import threading
import time

import jsonpickle
from flask import Flask, Response, current_app, request
from flask_socketio import SocketIO


# Disable logging of http requests in console
logging.getLogger('werkzeug').setLevel(logging.ERROR)
logging.getLogger('socketio').setLevel(logging.ERROR)
logging.getLogger('engineio').setLevel(logging.ERROR)
logging.basicConfig(format='%(asctime)s %(message)s', datefmt='%Y-%m-%d %H:%M:%S [webserver]', level=logging.INFO)

class Webserver:

    def __init__(self):
        self.state = {}
        self.somethingtoemit = True
        self.lastemit = 0

        app = Flask(__name__, static_folder='../build', static_url_path='')
        app.config['PROPAGATE_EXCEPTIONS'] = True
        self.socketio = SocketIO(app, cors_allowed_origins="*")

        app.add_url_rule('/', 'root', self.root)
        app.add_url_rule('/state', 'get_state', self.get_state, methods=['GET'])
        app.add_url_rule('/state', 'post_state', self.post_state, methods=['POST'])

        thread = threading.Thread(target=self.emit_state_thread)
        thread.daemon = True
        thread.start()

        logging.info("Starting server on port 1097")
        self.socketio.run(app, host="0.0.0.0", port=1097)

    def root(self):
        return current_app.send_static_file('index.html')

    def post_state(self):
        update(self.state, request.json)
        self.somethingtoemit = True
        return Response(status=200)

    def get_state(self):
        return Response(response=self.to_json(), status=200, mimetype='application/json')

    def to_json(self):
        return jsonpickle.encode(self.state, unpicklable=False, warn=True)

    def emit_state_thread(self):
        while 1:
            time.sleep(0.1)
            if self.socketio is None:
                continue
            timesincelastemit = time.time() - self.lastemit
            if timesincelastemit < 0.3:
                # Never emit state if last emitted state was less then 0.3 s ago
                continue

            if timesincelastemit > 1 or self.somethingtoemit:
                # Always emit if last emit was more then 1 second ago
                self.emit_state()

    def emit_state(self):
        self.socketio.emit('state', self.to_json(), broadcast=True)
        self.lastemit = time.time()
        self.somethingtoemit = False


Webserver()
