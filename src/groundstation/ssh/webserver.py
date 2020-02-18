import logging

from flask import Flask, request
from flask_cors import CORS
from waitress import serve

from .sshclient import SSHClient


class Webserver:

    def __init__(self,
                 ssh,  # type: SSHClient
                 ):
        self.app = Flask(__name__)
        self.app.config['PROPAGATE_EXCEPTIONS'] = True
        CORS(self.app)
        self.ssh = ssh

    def start(self):
        # Disable logging of http requests in console
        log = logging.getLogger('werkzeug')
        log.setLevel(logging.ERROR)

        self.app.add_url_rule('/runcommand', 'runcommand', self.exec_ssh_command, methods=['POST'])

        logging.info("Starting server on port 1098")
        serve(self.app, host="0.0.0.0", port=1098)

    def exec_ssh_command(self):
        jsondata = request.get_json()
        if 'command' not in jsondata:
            return "json field 'command' not found", 400
        try:
            cmd = jsondata['command']
            logging.error("executing command:" + str(cmd))
            statuscode, output = self.ssh.run_command(cmd)
            return {
                'statuscode': statuscode,
                'output': output
            }, 200
        except Exception, e:
            logging.error("error executing command:" + str(e))
            return str(e), 500
