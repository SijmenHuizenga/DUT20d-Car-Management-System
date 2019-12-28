import time
import logging
from flask import Flask, send_from_directory, request, abort
import socketio

app = Flask(__name__, static_folder='../../operator/build/', static_url_path='')


class Webserver:

    def __init__(self, state, logbook, ssh, recorder):
        self.state = state
        self.app = None
        self.logbook = logbook
        self.ssh = ssh
        self.recorder = recorder

    def start(self):
        # Disable logging of http requests in console
        log = logging.getLogger('werkzeug')
        log.setLevel(logging.ERROR)

        sio = socketio.Server(async_mode='threading')
        app.config['PROPAGATE_EXCEPTIONS'] = True
        app.wsgi_app = socketio.WSGIApp(sio, app.wsgi_app)
        self.state.set_socketio(sio)

        app.add_url_rule('/', 'root', self.root)
        app.add_url_rule('/state', 'state', self.state)
        app.add_url_rule('/logbook', 'logbook_create', self.logbook_create, methods=['POST'])
        app.add_url_rule('/logbook/<int:rowid>', 'logbook_update', self.logbook_update, methods=['PUT'])
        app.add_url_rule('/rebootluke', 'reboot_luke', self.reboot_luke)
        app.add_url_rule('/recording/toggletopic', 'recording_toggle', self.recording_toggle, methods=['PUT'])
        app.add_url_rule('/recording/filename', 'recording_setfilename', self.recording_setfilename, methods=['PUT'])
        app.add_url_rule('/recording/start', 'recording_start', self.recording_start, methods=['PUT'])
        app.add_url_rule('/recording/stop', 'recording_stop', self.recording_stop, methods=['PUT'])

        app.run(threaded=True)

    def root(self):
        return self.app.send_static_file('index.html')

    def state(self):
        return self.state.state

    def logbook_create(self):
        try:
            content = request.get_json()
            if 'text' not in content:
                abort(400)
            if 'timestamp' not in content:
                timestamp = time.time()
            else:
                timestamp = request.json.timestamp
            self.logbook.add_line(timestamp, content['text'], content['source'])
            return "ok", 201
        except Exception, e:
            print "error while handling /logbook request:", e
            abort(500, e)

    def logbook_update(self, rowid):
        try:
            changes = request.get_json()
            if 'text' in changes:
                self.logbook.update_line_text(rowid, changes['text'])

            if 'timestamp' in changes:
                self.logbook.update_line_timestamp(rowid, changes['timestamp'])
            return "ok", 201
        except Exception, e:
            print "error while handling /logbook request:", e
            abort(500, e)

    def reboot_luke(self):
        try:
            code, output = self.ssh.run_command("sudo reboot")
            if code != 0:
                raise Exception("Return code from 'sudo reboot' was " + str(code) + ": " + output)
        except Exception, e:
            return str(e), 400
        return "ok: " + output, 200

    def recording_toggle(self):
        try:
            json = request.get_json()
            if 'topicname' not in json or 'selected' not in json:
                abort(400)

            self.recorder.set_topic(json['topicname'], json['selected'])
            return "ok", 201
        except Exception, e:
            print "error while updating seleced topics:", e
            abort(500, e)

    def recording_setfilename(self):
        try:
            json = request.get_json()
            if 'filename' not in json:
                abort(400)

            self.recorder.set_filename(json['filename'])
            return "ok", 200
        except Exception, e:
            print "error while updating filename:", e
            abort(500, e)

    def recording_start(self):
        try:
            self.recorder.recording_start()
            return "ok", 200
        except Exception, e:
            print "error starting recording:", e
            abort(500, e)

    def recording_stop(self):
        try:
            self.recorder.recording_stop()
            return "ok", 200
        except Exception, e:
            print "error stopping recording:", e
            abort(500, e)
