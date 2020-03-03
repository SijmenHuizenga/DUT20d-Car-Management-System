#!/usr/bin/env python
import logging

from groundstation.rosnode.pinger import Pinger
from groundstation.rosnode.rosnode import RosNode
from groundstation.rosnode.rosrecording import RosRecorder
from groundstation.rosnode.webserver import Webserver
from groundstation.utils import loadconfig

logging.basicConfig(format='%(asctime)s %(message)s', datefmt='%Y-%m-%d %H:%M:%S [ros]', level=logging.INFO)

config = loadconfig()
Pinger(config['ip']).start()

rosrecorder_ = RosRecorder()

Webserver(rosrecorder_).start()

RosNode(rosrecorder_).run()
