import datetime
import logging
import sqlite3
import time

import jsonpickle as jsonpickle
import requests
from flask import request, Response

from groundstation.models import LogbookLine
from groundstation.utils import dict_factory


class Logbook:

    def __init__(self, slackurl):
        self.database = sqlite3.connect('/var/cms/logbook-%s.db' % datetime.date.today(), check_same_thread=False)
        self.database.row_factory = dict_factory
        self.create_schema()
        self.slackurl = slackurl

    def get(self):
        try:
            return Response(response=jsonpickle.encode(self.get_all(), unpicklable=False, warn=True), status=200, mimetype='application/json')
        except Exception, e:
            print "error while handling /logbook request:", e
            return str(e), 500

    def create(self):
        try:
            content = request.get_json()
            if 'text' not in content or 'source' not in content:
                return "json field 'text' or 'source' not found", 400
            if 'timestamp' not in content:
                timestamp = time.time()
            else:
                timestamp = content['timestamp']
            self.add_line(timestamp, content['text'], content['source'])
            return self.get()
        except Exception, e:
            print "error while handling /logbook request:", e
            return str(e), 500

    def update(self, rowid):
        try:
            changes = request.get_json()
            if 'text' in changes:
                self.update_line_text(rowid, changes['text'])

            if 'timestamp' in changes:
                self.update_line_timestamp(rowid, changes['timestamp'])
            return self.get()
        except Exception, e:
            print "error while handling /logbook request:", e
            return str(e), 500

    def slack(self, rowid):
        try:
            line = self.get_line(rowid)
            r = requests.post(url=self.slackurl,
                              data=jsonpickle.encode({
                                  "text": datetime.datetime.fromtimestamp(line.timestamp).strftime('%Y-%m-%d %H:%M:%S') + ": " + line.text
                              }, unpicklable=False, warn=True),
                              headers={'content-type': 'application/json'})
            if r.status_code != 200:
                return str("Slack returned status code " + r.status_code), 503
            else:
                return str("ok"), 200
        except Exception, e:
            print "error while handling /logbook request:", e
            return str(e), 500

    def create_schema(self):
        with self.database:
            self.database.execute('CREATE TABLE IF NOT EXISTS logbook (timestamp FLOAT, text TEXT, source VARCHAR(255))')
            logging.info("Logbook database ready to go")

    def get_all(self):
        cursor = self.database.cursor()
        cursor.execute("SELECT rowid, * FROM logbook", {})
        out = cursor.fetchall()
        if out is None:
            return []
        cursor.close()
        return map(self.logline_fromdict, out)

    def get_line(self, rowid):
        cursor = self.database.cursor()
        cursor.execute("SELECT rowid, * FROM logbook WHERE rowid=:rowid", {'rowid': rowid})
        out = cursor.fetchone()
        cursor.close()
        if out is None:
            raise Exception("Line does not exit")
        return self.logline_fromdict(out)

    def logline_fromdict(self, d):
        return LogbookLine(d['timestamp'], d['text'], d['source'], d['rowid'])

    def add_line(self, timestamp, text, source):
        assert text, "Text cannot be empty"
        with self.database:
            self.database.execute('INSERT INTO logbook VALUES (:timestamp, :text, :source)', {
                'timestamp': timestamp,
                'text': text,
                'source': source
            })

        logging.info('Added line: [%s] %s' % (source, text))

    def update_line_text(self, rowid, text):
        assert text, "Text cannot be empty"
        with self.database:
            self.database.execute("UPDATE logbook SET text=:text WHERE rowid=:rowid", {
                'text': text,
                'rowid': rowid
            })

        logging.info('Updated line text rowid %d to "%s"' % (rowid, text))

    def update_line_timestamp(self, rowid, timestamp):
        with self.database:
            self.database.execute("UPDATE logbook SET timestamp=:timestamp WHERE rowid=:rowid", {
                'rowid': rowid,
                'timestamp': timestamp
            })
        logging.info('Updated line timestmap rowid %d to "%f"' % (rowid, timestamp))


