import datetime
import sqlite3
import time
import logging

import jsonpickle as jsonpickle
from flask import Flask, abort, request, Response
from flask_cors import CORS
from waitress import serve

logging.basicConfig(format='%(asctime)s %(message)s', datefmt='%Y-%m-%d %H:%M:%S [logbookserver]', level=logging.INFO)


class LogbookLine:
    def __init__(self, timestamp, text, source, rowid=None):
        # The timestamp on which the line resides.
        # If the line is updated the old value is replaced by the new.
        self.timestamp = timestamp  # type: float

        # The multiline and markdown supported text.
        self.text = text  # type: str

        # Who made this logline? This field keeps track of who issued the line.
        self.source = source  # type: str

        # The id of the row in the database. Optional.
        self.rowid = rowid  # type: int


class Logbook:

    def __init__(self):
        self.database = sqlite3.connect('logbook-%s.db' % datetime.date.today(), check_same_thread=False)
        self.database.row_factory = dict_factory
        self.create_schema()

    def get(self):
        try:
            return Response(response=jsonpickle.encode(self.get_all(), unpicklable=False, warn=True), status=200, mimetype='application/json')
        except Exception, e:
            print "error while handling /logbook request:", e
            abort(500, e)

    def create(self):
        try:
            content = request.get_json()
            if 'text' not in content or 'source' not in content:
                abort(400)
            if 'timestamp' not in content:
                timestamp = time.time()
            else:
                timestamp = content['timestamp']
            self.add_line(timestamp, content['text'], content['source'])
            return self.get()
        except Exception, e:
            print "error while handling /logbook request:", e
            abort(500, e)

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
            abort(500, e)

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


def dict_factory(cursor, row):
    d = {}
    for idx, col in enumerate(cursor.description):
        d[col[0]] = row[idx]
    return d


logbook = Logbook()

app = Flask(__name__)
CORS(app)
app.config['PROPAGATE_EXCEPTIONS'] = True
app.add_url_rule('/logbook', 'logbook_get', logbook.get, methods=['GET'])
app.add_url_rule('/logbook', 'logbook_create', logbook.create, methods=['POST'])
app.add_url_rule('/logbook/<int:rowid>', 'logbook_update', logbook.update, methods=['PUT'])
serve(app, host="0.0.0.0", port=1095)
