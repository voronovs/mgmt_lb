import time
from pymavlink import mavutil
import logging
import math


class CMavlinkSerialConnection:
    def __init__(self, port, baud, txCounterDict=None, rxCounterDict=None):
        self.conn = port
        self.baud = baud
        self.REOPEN_TIMEOUT = 1.0
        self.reopenTS = time.time() - 1
        self.isConnected = False
        self.master = None
        self.open()
        self._txCounterDict = txCounterDict if txCounterDict else {}
        self._rxCounterDict = rxCounterDict if rxCounterDict else {}
        self._rxCounter = 0
        self._txCounter = 0

    def tx_increment(self, conn_string):
        self._txCounterDict[conn_string] = self._txCounterDict[conn_string] + 1

    def get_stats(self, rate_scale: float) -> str:
        return 'RX/TX {0} ({1} / {2}) \t' \
            .format(
                self.conn,
                math.ceil(self._rxCounter * rate_scale),
                math.ceil(self._txCounter * rate_scale))

    def reset_stats(self):
        self._rxCounter = 0
        self._txCounter = 0
        self._rxCounterDict[self.conn] = 0
        self._txCounterDict[self.conn] = 0

    def open(self):
        if self.reopenTS < time.time():
            self.reopenTS = time.time() + self.REOPEN_TIMEOUT
            try:
                self.master = mavutil.mavlink_connection(self.conn, baud=self.baud, input=False)
                self.isConnected = True
            except Exception as e:
                logging.error('Error (%s) opening serial port: %s:%s', e, self.conn, self.baud)
                self.isConnected = False

    def send_buf(self, msgBuf):
        self._txCounterDict[self.conn] = self._txCounterDict[self.conn] + 1
        self._txCounter += 1
        self.master.write(msgBuf)

    def send(self, msg):
        self._txCounterDict[self.conn] = self._txCounterDict[self.conn] + 1
        self._txCounter += 1
        self.master.mav.send(msg)

        # TODO
        #  Try https://github.com/peterbarker/dronekit-python
        #  /blob/eef45b0cb3e251e6bb4cab56060f91856cd54a02/examples/multivehicle/mavlink_hub.py#L84
        #  self.master.mav.write(msg.get_msgbuf())

    def spin(self):
        # TODO
        #   File "mgmt.py", line 204, in recv
        #     msg = self.master.recv_match(blocking=False)
        #   File "/usr/local/lib/python2.7/site-packages/pymavlink/mavutil.py", line 457, in recv_match
        #     m = self.recv_msg()
        #   File "/usr/local/lib/python2.7/site-packages/pymavlink/mavutil.py", line 420, in recv_msg
        #     s = self.recv(n)
        #   File "/usr/local/lib/python2.7/site-packages/pymavlink/mavutil.py", line 962, in recv
        #     ret = self.port.read(n)
        #   File "/usr/local/lib/python2.7/site-packages/serial/serialposix.py", line 509, in read
        #     raise SerialException('read failed: {}'.format(e))
        #   serial.serialutil.SerialException: read failed: [Errno 6] Device not configured

        try:
            msg = self.master.recv_match(blocking=False)

            if msg:
                if msg.get_type() == "BAD_DATA":
                    pass
                elif msg.get_type() == "HEARTBEAT":
                    self._rxCounterDict[self.conn] = self._rxCounterDict[self.conn] + 1
                    self._rxCounter += 1
                    return msg
                else:
                    self._rxCounterDict[self.conn] = self._rxCounterDict[self.conn] + 1
                    self._rxCounter += 1
                    return msg
            return 0
        except Exception as e:
            self.open()
