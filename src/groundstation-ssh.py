#!/usr/bin/env python

import logging

from groundstation.ssh.sshclient import SSHClient
from groundstation.ssh.sshping import SSHPing
from groundstation.ssh.systemdservices import SystemdServices
from groundstation.ssh.webserver import Webserver
from groundstation.utils import loadconfig

logging.basicConfig(format='%(asctime)s %(message)s', datefmt='%Y-%m-%d %H:%M:%S [ssh]', level=logging.INFO)

config = loadconfig()
client = SSHClient(host=config['ip'], username=config['username'], password=config['password'])

SSHPing(client).start()
SystemdServices(client).start()

Webserver(client).start()
