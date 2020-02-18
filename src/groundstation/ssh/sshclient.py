import logging
import socket
import threading

import ssh2
from ssh2.error_codes import LIBSSH2_ERROR_EAGAIN
from ssh2.session import Session


class SSHClient:

    def __init__(self,
                 host,  # type: str
                 username,  # type: str
                 password  # type: str
                 ):
        self.host = host
        self.username = username
        self.password = password
        self.session = None
        self.sock = None
        self.lock = threading.Lock()

    def run_command(self, command):
        try:
            self.lock.acquire()
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
            if self.sock is not None:
                self.sock.close()
            self.sock = None
            logging.exception(err)
            raise Exception("SSH Error: " + type(err).__name__ + ": " + str(err))
        finally:
            self.lock.release()

    def ensure_transport(self):
        if self.sock is None or self.session is None:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            # 4 seconds
            self.sock.settimeout(4)
            self.sock.connect(("192.168.1.1", 22))

            self.session = Session()
            # 5 seconds
            self.session.set_timeout(5000)
            self.session.handshake(self.sock)
            self.session.userauth_password(self.username, self.password)
            # self.session.keepalive_config(False, 3)
            logging.info("(re)opened transport")

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