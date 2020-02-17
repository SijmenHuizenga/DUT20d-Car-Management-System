#!/usr/bin/env python
import logging

from flask import Flask
from flask_cors import CORS
from waitress import serve

from groundstation.logbook.logbook import Logbook

logging.basicConfig(format='%(asctime)s %(message)s', datefmt='%Y-%m-%d %H:%M:%S [logbook]', level=logging.INFO)

logbook = Logbook()

app = Flask(__name__)
CORS(app)
app.config['PROPAGATE_EXCEPTIONS'] = True
app.add_url_rule('/logbook', 'logbook_get', logbook.get, methods=['GET'])
app.add_url_rule('/logbook', 'logbook_create', logbook.create, methods=['POST'])
app.add_url_rule('/logbook/<int:rowid>', 'logbook_update', logbook.update, methods=['PUT'])
serve(app, port=1095)
