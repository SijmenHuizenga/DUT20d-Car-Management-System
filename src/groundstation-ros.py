#!/usr/bin/env python
import commands
import logging
import os
import sys

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

if os.environ.get('ROS_IP') is None:
    myip = commands.getoutput("ip addr show | grep 'inet ' | awk '{print $2}' | cut -f1 -d'/' | grep '192.168.1.' | head -n 1")
    if myip is '':
        print "No ROS_IP set and could not find current ip in range 192.168.1.*"
        sys.exit(1)
    print("Setting ROS_IP to " + myip)
    os.environ['ROS_IP'] = myip

if os.environ.get('ROS_MASTER_URI') is None:
    print("ROS_MASTER_URI unset, setting it to http://" + config['host'] + ":11311")
    os.environ['ROS_MASTER_URI'] = "http://" + config['host'] + ":11311"

RosNode(rosrecorder_).run()
