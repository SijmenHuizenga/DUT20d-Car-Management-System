import time
import logging
from flask import Flask, send_from_directory, request, abort
import socketio

from state import statemanager as state
import logbook
logbook = logbook.Logbook()

# Disable logging of http requests in console
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

sio = socketio.Server(async_mode='threading')
app = Flask(__name__, static_folder='../../operator/build/', static_url_path='')
app.wsgi_app = socketio.WSGIApp(sio, app.wsgi_app)
state.set_sio(sio)


def start_server():
    app.run(threaded=True)


@app.route('/')
def root():
    return app.send_static_file('index.html')


@app.route('/state')
def home():
    return state.state


@app.route('/logbook', methods=['POST'])
def logbook_create():
    try:
        content = request.get_json()
        if 'text' not in content:
            abort(400)
        if 'timestamp' not in content:
            timestamp = time.time()
        else:
            timestamp = request.json.timestamp
        logbook.add_line(timestamp, content['text'], content['source'])
        return "ok", 201
    except Exception, e:
        print "error while handling /logbook request:", e
        abort(500, e)


@app.route('/logbook/<int:rowid>', methods=['PUT'])
def logbook_update(rowid):
    try:
        changes = request.get_json()
        if 'text' in changes:
            logbook.update_line_text(rowid, changes['text'])

        if 'timestamp' in changes:
            logbook.update_line_timestamp(rowid, changes['timestamp'])
        return "ok", 201
    except Exception, e:
        print "error while handling /logbook request:", e
        abort(500, e)
