import datetime
import logging
import time
from peewee import *

import jsonpickle as jsonpickle
from flask import request, Response


db = SqliteDatabase('/var/cms/logbook-%s.db' % datetime.date.today())


class LogbookLine(Model):
    rowid = AutoField()
    timestamp = FloatField()
    text = CharField()
    lastupdated = FloatField()

    # Every slack message is uniquely identified by it's timestamp and channel.
    # When we post a message to slack the timestamp is stored here.
    # If timestamp is None the message is not yet synced to Slack.
    slacktimestamp = CharField(null=True)

    # The timestamp when the line was last synchronized with slack.
    # If this field is None the message is not yet synced to Slack.
    # If this field's value is unequal to lastupdated it indicates that the message was initially synced with Slack but
    # the text was changed since the last sync.
    slacklastupdated = FloatField(null=True)

    class Meta:
        database = db

    def get_slack_msg(self, showtimestamp):
        out = ""

        if showtimestamp:
            out += datetime.datetime.fromtimestamp(self.timestamp).strftime('%H:%M:%S') + ": "

        return out + self.text


db.connect()
db.create_tables([LogbookLine])


class Logbook:

    def __init__(self, slackclient, slackchannel):
        self.slackclient = slackclient
        self.slackchannel = slackchannel

    def get(self):
        try:
            return self.jsonresponse(list(LogbookLine.select().dicts()))
        except Exception, e:
            print "error while handling /logbook request:", e
            return str(e), 500

    def create(self):
        try:
            content = request.get_json()
            if 'text' not in content:
                return "json field 'text' not found", 400
            if 'timestamp' not in content:
                timestamp = time.time()
            else:
                timestamp = content['timestamp']

            line = LogbookLine.create(timestamp=timestamp, text=content['text'],
                                      lastupdated=time.time(), slacklastupdated=None, slacktimestamp=None)

            self.synclinetoslack(line)
            return self.get()
        except Exception, e:
            print "error while handling /logbook request:", e
            return str(e), 500

    def update(self, rowid):
        try:
            changes = request.get_json()
            if 'text' not in changes:
                return "Bad request", 400

            line = LogbookLine.get_by_id(rowid)
            line.text = changes['text']
            line.lastupdated = time.time()
            line.save()

            self.synclinetoslack(line)

            return self.get()
        except Exception, e:
            print "error while handling /logbook request:", e
            return str(e), 500

    def fullslacksync(self):
        successes = 0
        fails = 0
        for line in LogbookLine.select():
            if line.lastupdated != line.slacklastupdated:
                if self.synclinetoslack(line):
                    successes += 1
                else:
                    fails += 1

        return self.jsonresponse({
            "logbook": list(LogbookLine.select().dicts()),
            "syncfailedlines": fails,
            "syncsuccesslines": successes,
        })

    def synclinetoslack(self, line):
        try:
            if line.slacktimestamp is None:
                response = self.slackclient.api_call(
                    "chat.postMessage",
                    channel=self.slackchannel,
                    text=line.get_slack_msg(time.time() - line.timestamp > 5),
                    parse="full",
                    link_names=True,
                )
            else:
                response = self.slackclient.api_call(
                    "chat.update",
                    ts=line.slacktimestamp,
                    channel=self.slackchannel,
                    text=line.get_slack_msg(float(line.slacktimestamp) - line.timestamp > 5),
                    parse="full",
                    link_names=True,
                )

            if response['ok']:
                line.slacklastupdated = line.lastupdated
                line.slacktimestamp = response['ts']
                line.save()
                return True
            else:
                logging.error("Logline slack sync failed: %s" % response['error'])
                return False
        except Exception, e:
            logging.error("Logline slack sync failed: %s" % str(e))
            return False


    def jsonresponse(self, json):
        return Response(response=jsonpickle.encode(json, unpicklable=False, warn=True), status=200,
                        mimetype='application/json')
