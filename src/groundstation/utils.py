import collections
import json
import logging
import os
import sys

import jsonpickle
import requests
import yaml


def add_logline(timestamp, text, source):
    r = requests.post(url="http://localhost:1095/logbook", data=json.dumps({
        'timestamp': timestamp,
        'text': text,
        'source': source
    }), headers={'content-type': 'application/json'})
    if r.status_code != 201:
        logging.info(r.text)
    else:
        logging.error("[%s] %s" % (source, text))


def sendstate(changeset):
    try:
        r = requests.post(url="http://localhost:1097/state",
                          data=jsonpickle.encode(changeset, unpicklable=False, warn=True),
                          headers={'content-type': 'application/json'})
        if r.status_code != 200:
            logging.error("Error while sending state update to webserver: " + r.text)
        else:
            logging.info("Sent state to webserver keys: " + ", ".join(list(changeset)))
    except Exception as e:
        logging.error(e)


def is_topic_ignored(topicname):
    if topicname.startswith("/rosout"):
        return True
    return False


def is_node_ignored(nodename):
    if nodename.startswith("/ros"):
        return True
    return False


def dict_factory(cursor, row):
    d = {}
    for idx, col in enumerate(cursor.description):
        d[col[0]] = row[idx]
    return d


def update(d, u):
    for k, v in u.items():
        if isinstance(v, collections.abc.Mapping):
            d[k] = update(d.get(k, {}), v)
        else:
            d[k] = v
    return d


def load_yaml(filename):
    with open(filename, 'r') as stream:
        try:
            return yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            sys.exit(exc)


def loadconfig():
    return load_yaml(os.path.join(os.path.dirname(__file__), "../../config/car.yaml"))


def loadsecrets():
    return load_yaml("/var/cms-keys.yaml")
