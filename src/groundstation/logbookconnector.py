import requests
import json


def add_logline(timestamp, text, source):
    r = requests.post(url="http://localhost:1095/logbook", data=json.dumps({
        'timestamp': timestamp,
        'text': text,
        'source': source
    }), headers={'content-type': 'application/json'})
    if r.status_code != 201:
        print r.text
    else:
        print "[%s] %s" % (source, text)
