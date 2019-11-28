import sys
import time
from flask import Flask, send_from_directory, request, abort
from state import statemanager as state

app = Flask(__name__, static_folder='../../operator/build/', static_url_path='')


def start_server():
    app.run()


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
        if 'timestamp' not in content:
            timestamp = time.time()
        else:
            timestamp = request.json.timestamp
        state.add_logbook_line(timestamp, content['text'], content['source'])
        return "ok", 201
    except Exception, e:
        print "error while handling /logbook request:", e
        abort(500, e)
