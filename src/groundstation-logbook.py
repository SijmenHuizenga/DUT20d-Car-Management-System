#!/usr/bin/env python
import logging

from slackclient import SlackClient
from flask import Flask
from flask_cors import CORS
from waitress import serve

from groundstation.logbook.logbook import Logbook
from groundstation.utils import loadsecrets, loadconfig

logging.basicConfig(format='%(asctime)s %(message)s', datefmt='%Y-%m-%d %H:%M:%S [logbook]', level=logging.INFO)

config = loadconfig()
secrets = loadsecrets()
sc = SlackClient(secrets['slackapitoken'])
logbook = Logbook(sc, config['slackchannel'])

app = Flask(__name__)
CORS(app)
app.config['PROPAGATE_EXCEPTIONS'] = True
app.add_url_rule('/logbook', 'logbook_get', logbook.get, methods=['GET'])
app.add_url_rule('/logbook', 'logbook_create', logbook.create, methods=['POST'])
app.add_url_rule('/logbook/<int:rowid>', 'logbook_update', logbook.update, methods=['PUT'])
app.add_url_rule('/logbook/sync', 'logbook_fullslacksync', logbook.fullslacksync, methods=['GET'])
serve(app, port=1095)
