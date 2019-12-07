import paramiko
import threading
import time


class SSHClient:

    def __init__(self, host, state, username, password):
        self.host = host
        self.state = state
        self.username = username
        self.password = password
        self.transport = None

        self.start()

    def start(self):
        thread = threading.Thread(target=self.ping_forever)
        thread.daemon = True
        thread.start()

    def run_command(self, command, connection_timeout=2, execution_timeout=2):
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

            (exitcode, uptimestr) = self.run_command("uptime -p")
            if exitcode != 0:
                raise Exception("uptime exit code was not 0")
            connected = self.transport.is_active()
        except Exception, e:
            uptimestr = str(e)
            connected = False
        self.state.update({
            'ssh': {
                'connected': connected,
                'uptime': uptimestr,
                'lastping': time.time()
            },
        })
