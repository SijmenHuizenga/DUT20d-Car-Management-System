#!/usr/bin/env python
import logging

from flask import Flask
from flask_cors import CORS
from waitress import serve

from groundstation.logbook.logbook import Logbook
from groundstation.utils import loadsecrets

logging.basicConfig(format='%(asctime)s %(message)s', datefmt='%Y-%m-%d %H:%M:%S [logbook]', level=logging.INFO)

config = loadsecrets()

logbook = Logbook(config['slack-webhook-url'])

app = Flask(__name__)
CORS(app)
app.config['PROPAGATE_EXCEPTIONS'] = True
app.add_url_rule('/logbook', 'logbook_get', logbook.get, methods=['GET'])
app.add_url_rule('/logbook', 'logbook_create', logbook.create, methods=['POST'])
app.add_url_rule('/logbook/<int:rowid>', 'logbook_update', logbook.update, methods=['PUT'])
app.add_url_rule('/logbook/<int:rowid>/slack', 'logbook_slack', logbook.slack, methods=['POST'])
serve(app, port=1095)
