import threading
import time
import socket

from ssh2.session import Session
from ssh2.error_codes import LIBSSH2_ERROR_EAGAIN
import ssh2

from .state import State


class SSHClient:

    def __init__(self,
                 host,  # type: str
                 state,  # type: State
                 username,  # type: str
                 password  # type: str
                 ):
        self.host = host
        self.state = state
        self.username = username
        self.password = password
        self.session = None
        self.sock = None

    def start(self):
        thread = threading.Thread(target=self.ping_forever)
        thread.daemon = True
        thread.start()

    def run_command(self, command, timeout=2):
        try:
            self.ensure_transport()
            channel = self.session.open_session()
            while channel == LIBSSH2_ERROR_EAGAIN:
                raise ssh2.exceptions.ChannelError("Channel could not be opened")
            channel.execute(command)
            status = channel.wait_eof()
            ssh2.utils.handle_error_codes(status)
            channel.close()
            channel.wait_closed()
            output = self.receive(channel)
            statuscode = channel.get_exit_status()
            return statuscode, output
        except ssh2.exceptions.SSH2Error, err:
            self.sock.close()
            self.sock = None
            raise Exception("SSH Error: " + type(err).__name__)


    def ensure_transport(self):
        if self.sock is None or self.session is None:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(5)
            self.sock.connect((self.host, 22))

            self.session = Session()
            self.session.keepalive_config(True, 2)
            self.session.set_timeout(5000)
            self.session.handshake(self.sock)
            self.session.userauth_password(self.username, self.password)

    def receive(self, channel):
        data = ""
        while True:
            size, x = channel.read(size=1024*8)
            if size <= 0:
                break
            data += x
        while True:
            size, x = channel.read_stderr(size=1024*8)
            if size <= 0:
                break
            data += x
        return data

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
            (exitcode, uptimestr) = self.run_command("uptime -p")
            if exitcode != 0:
                raise Exception("uptime exit code was not 0")
            connected = True
        except Exception, e:
            uptimestr = str(e)
            connected = False

        self.state.ssh.connected = connected
        self.state.ssh.uptime = uptimestr
        self.state.ssh.lastping = time.time()
