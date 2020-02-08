import logging
import sys
import traceback

from flask import Flask, request, abort, Response, current_app
from flask_socketio import SocketIO

from .rosrecording import RosRecorder
from .sshclient import SSHClient
from .state import State

app = Flask(__name__, static_folder='../../build', static_url_path='')


class Webserver:

    def __init__(self,
                 state,  # type: State
                 ssh,  # type: SSHClient
                 recorder  # type: RosRecorder
                 ):
        self.state = state
        self.app = None
        self.ssh = ssh
        self.recorder = recorder

    def start(self):
        # Disable logging of http requests in console
        log = logging.getLogger('werkzeug')
        log.setLevel(logging.ERROR)

        app.config['PROPAGATE_EXCEPTIONS'] = True
        socketio = SocketIO(app, cors_allowed_origins="*")
        self.state.set_socketio(socketio)

        app.add_url_rule('/', 'root', self.root)
        app.add_url_rule('/state', 'state', self.request_state, methods=['GET'])
        app.add_url_rule('/runcommand', 'runcommand', self.exec_ssh_command, methods=['POST'])
        app.add_url_rule('/rebootluke', 'reboot_luke', self.reboot_luke, methods=['POST'])
        app.add_url_rule('/recording/toggletopic', 'recording_toggle', self.recording_toggle, methods=['PUT'])
        app.add_url_rule('/recording/filename', 'recording_setfilename', self.recording_setfilename, methods=['PUT'])
        app.add_url_rule('/recording/start', 'recording_start', self.recording_start, methods=['PUT'])
        app.add_url_rule('/recording/stop', 'recording_stop', self.recording_stop, methods=['PUT'])

        print app.static_folder
        socketio.run(app, host="0.0.0.0")

    def root(self):
        return current_app.send_static_file('index.html')

    def request_state(self):
        return Response(response=self.state.to_json(), status=200, mimetype='application/json')

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
            traceback.print_exc(file=sys.stdout)
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

    def exec_ssh_command(self):
        json = request.get_json()
        if 'command' not in json:
            abort(400)
        try:
            statuscode, output = self.ssh.run_command(json['command'])
            return {
                'statuscode': statuscode,
                'output': output
            }
        except Exception, e:
            print "error executing command:", e
            abort(500, e)
