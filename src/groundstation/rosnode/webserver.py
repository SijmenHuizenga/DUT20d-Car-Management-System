import logging
import sys
import threading
import traceback

from flask import Flask, request
from flask_cors import CORS
from waitress import serve

from .rosrecording import RosRecorder


class Webserver:

    def __init__(self,
                 recorder  # type: RosRecorder
                 ):
        self.app = Flask(__name__)
        CORS(self.app)
        self.recorder = recorder

    def start(self):
        thread = threading.Thread(target=self.run)
        thread.daemon = True
        thread.start()

    def run(self):
        # Disable logging of http requests in console
        log = logging.getLogger('werkzeug')
        log.setLevel(logging.ERROR)

        self.app.config['PROPAGATE_EXCEPTIONS'] = True
        self.app.add_url_rule('/recording/settopics', 'recording_toggle', self.recording_toggle, methods=['PUT'])
        self.app.add_url_rule('/recording/filename', 'recording_setfilename', self.recording_setfilename, methods=['PUT'])

        logging.info("Starting server on port 1099")
        serve(self.app, host="0.0.0.0", port=1099)

    def recording_toggle(self):
        try:
            json = request.get_json()
            if 'topicnames' not in json or 'selected' not in json:
                return "json field 'topicname' or 'selected' not found", 400

            self.recorder.set_topics(json['topicnames'], json['selected'])
            return "ok", 201
        except Exception, e:
            traceback.print_exc(file=sys.stdout)
            logging.error("error while updating seleced topics:" + str(e))
            return str(e), 500

    def recording_setfilename(self):
        try:
            json = request.get_json()
            if 'filename' not in json:
                return "json field 'filename' not found", 400

            self.recorder.set_filename(json['filename'])
            return "ok", 200
        except Exception, e:
            logging.error("error while updating filename:" + str(e))
            return str(e), 500

