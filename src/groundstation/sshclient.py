import paramiko
import threading
import time
from state import statemanager as state


class SSHClient:

    def __init__(self, host, username, password):
        self.host = host
        self.username = username
        self.password = password
        self.transport = None

        self.thread = threading.Thread(target=self.ping_forever)
        self.thread.daemon = True
        self.thread.start()

    def run_command(self, command, connection_timeout=2, execution_timeout=5):
        self.ensure_transport()
        channel = self.transport.open_session(timeout=connection_timeout)
        channel.set_combine_stderr(True)
        channel.settimeout(execution_timeout)
        channel.exec_command(command)
        output = self.receive(channel)
        statuscode = channel.recv_exit_status()
        channel.close()
        return statuscode, output

    def ensure_transport(self, timeout=3):
        if self.transport and self.transport.is_active():
            return
        if self.transport:
            self.transport.close()
        self.transport = paramiko.Transport((self.host, 22))
        self.transport.start_client(timeout=timeout)
        self.transport.auth_password(self.username, self.password)
        self.transport.set_keepalive(10)

    def receive(self, channel):
        data = ""
        while True:
            x = channel.recv(1024)
            if len(x) == 0:
                return data
            data += x

    def ping_forever(self):
        while 1:
            try:
                self.ping()
            except Exception, e:
                print("[sshclient] Someting unexpected happend while checking health: %s" % str(e))
            time.sleep(1)

    def ping(self):
        try:
            self.ensure_transport()
            self.transport.send_ignore()
            state.update({
                'ssh': {
                    'connected': self.transport.is_active()
                }
            })
        except Exception:
            state.update({'ssh': {'connected': False}})


ssh = None


def init(host, user, password):
    global ssh
    ssh = SSHClient(host, user, password)
    print "ssh client initialized"
