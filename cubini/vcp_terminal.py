# Copyright 2009–2017 Wander Lairson Costa
# Copyright 2009–2020 PyUSB contributors
#
#Redistribution and use in source and binary forms, with or without
#modification, are permitted provided that the following conditions are met:
#
#1. Redistributions of source code must retain the above copyright notice, this
#list of conditions and the following disclaimer.
#
#2. Redistributions in binary form must reproduce the above copyright notice,
#this list of conditions and the following disclaimer in the documentation
#and/or other materials provided with the distribution.
#
#3. Neither the name of the copyright holder nor the names of its contributors
#may be used to endorse or promote products derived from this software without
#specific prior written permission.
#
#THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
#ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
#WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
#FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
#DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
#SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
#OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# from https://github.com/pyusb/pyusb/blob/master/tools/vcp_terminal.py

import os
import sys
import time
import usb.core as usb
import threading
import logging
import queue

CDC_CMDS = {
    "SEND_ENCAPSULATED_COMMAND": 0x00,
    "GET_ENCAPSULATED_RESPONSE": 0x01,
    "SET_COMM_FEATURE": 0x02,
    "GET_COMM_FEATURE": 0x03,
    "CLEAR_COMM_FEATURE": 0x04,
    "SET_LINE_CODING": 0x20,
    "GET_LINE_CODING": 0x21,
    "SET_CONTROL_LINE_STATE": 0x22,
    "SEND_BREAK": 0x23,   # wValue is break time
}

class ComPort(object):

    def __init__(self, usb_device, start=True):
        self.device = usb_device
        self._isFTDI = False
        self._rxinterval = 0.005            # sec
        self._rxqueue = queue.Queue()
        self._rxthread = None
        self._rxactive = False
        self.baudrate = 9600
        self.parity = 0
        self.stopbits = 1
        self.databits = 8

        if os.name == 'nt':
            usb_device.set_configuration()
            cfg = usb_device.configurations()[0]
        else:
            cfg = usb_device.get_active_configuration()

        if self.device.idVendor == 0x0403:       # FTDI device
            self._isFTDI = True
            log.debug("Configuring as an FTDI device, no cmd itf")
            cmd_itfs = None
            data_itfs = list(usb.util.find_descriptor(
                cfg, find_all=True,
                custom_match=lambda e: (e.bInterfaceClass == 0xFF)))
            data_itf = data_itfs[0]
            itf_num = data_itf.bInterfaceNumber
        else:
            data_itfs = list(
                usb.util.find_descriptor(
                    cfg,
                    find_all=True,
                    custom_match=lambda e: (
                        e.bInterfaceClass == 0xA)))
            if not data_itfs:
                print("Unable to connect.  No data interfaces on device")
                exit()

            data_itf = data_itfs[0]
            cmd_itfs = list(
                usb.util.find_descriptor(
                    cfg,
                    find_all=True,
                    custom_match=lambda e: (
                        e.bInterfaceClass == 0x2)))
            itf_num = cmd_itfs[0].bInterfaceNumber
            if len(cmd_itfs) != len(data_itfs):
                log.debug("COM port data / command interface mismatch")

        ports = len(data_itfs)
        log.debug("found {0} COM port\n".format(ports))

        try:
            self.device.detach_kernel_driver(itf_num)
        except usb.USBError:
            pass
        except NotImplementedError:
            pass

        self._ep_in = usb.util.find_descriptor(
            data_itf, custom_match=lambda e: (
                e.bEndpointAddress & 0x80))
        self._ep_out = usb.util.find_descriptor(
            data_itf, custom_match=lambda e: not (
                e.bEndpointAddress & 0x80))

        if start:
            self._startRx()

    def _startRx(self):
        if self._rxthread is not None and (
                self._rxactive or self._rxthread.isAlive()):
            return
        self._rxactive = True
        self._rxthread = threading.Thread(target=self._read)
        self._rxthread.daemon = True
        self._rxthread.start()

    def _endRx(self):
        self._rxactive = False

    def _read(self):
        """ check ep for data, add it to queue and sleep for interval """
        while self._rxactive:
            try:
                rv = self._ep_in.read(self._ep_in.wMaxPacketSize)
                if self._isFTDI:
                    status = rv[:2] # FTDI prepends 2 flow control characters,
                    # modem status and line status of the UART
                    if status[0] != 1 or status[1] != 0x60:
                        log.info(
                            "USB Status: 0x{0:02X} 0x{1:02X}".format(
                                *status))
                    rv = rv[2:]
                for rvi in rv:
                    self._rxqueue.put(rvi)
            except usb.USBError as e:
                log.warn("USB Error on _read {}".format(e))
                return
            
            time.sleep(self._rxinterval)

    def _getRxLen(self):
        return self._rxqueue.qsize()
    rxlen = property(fget=_getRxLen)

    def readBytes(self):
        rx = []
        while not self._rxqueue.empty():
            rx.append(self._rxqueue.get())
        return rx

    def readText(self):
        return "".join(chr(c) for c in self.readBytes())

    def write(self, data):
        try:
            ret = self._ep_out.write(data)
        except usb.USBError as e:
            log.warn("USB Error on write {}".format(e))
            return

        if len(data) != ret:
            log.error(
                "Bytes written mismatch {0} vs {1}".format(
                    len(data), ret))
        else:
            log.debug("{} bytes written to ep".format(ret))

    def setControlLineState(self, RTS=None, DTR=None):
        ctrlstate = (2 if RTS else 0) + (1 if DTR else 0)
        if self._isFTDI:
            ctrlstate += (1 << 8) if DTR is not None else 0
            ctrlstate += (2 << 8) if RTS is not None else 0

        txdir = 0           # 0:OUT, 1:IN
        req_type = 2 if self._isFTDI else 1     # 0:std, 1:class, 2:vendor
        # 0:device, 1:interface, 2:endpoint, 3:other
        recipient = 0 if self._isFTDI else 1
        req_type = (txdir << 7) + (req_type << 5) + recipient

        wlen = self.device.ctrl_transfer(
            bmRequestType=req_type,
            bRequest=1 if self._isFTDI else CDC_CMDS["SET_CONTROL_LINE_STATE"],
            wValue=ctrlstate,
            wIndex=1 if self._isFTDI else 0,
            data_or_wLength=0)
        log.debug("Linecoding set, {}b sent".format(wlen))

    def setLineCoding(self, baudrate=None, parity=None,
                      databits=None, stopbits=None):
        sbits = {1: 0, 1.5: 1, 2: 2}
        dbits = {5, 6, 7, 8, 16}
        pmodes = {0, 1, 2, 3, 4}
        brates = {300, 600, 1200, 2400, 4800, 9600, 14400,
                  19200, 28800, 38400, 57600, 115200, 230400}

        if stopbits is not None:
            if stopbits not in sbits.keys():
                valid = ", ".join(str(k) for k in sorted(sbits.keys()))
                raise ValueError("Valid stopbits are " + valid)
            self.stopbits = stopbits

        if databits is not None:
            if databits not in dbits:
                valid = ", ".join(str(d) for d in sorted(dbits))
                raise ValueError("Valid databits are " + valid)
            self.databits = databits

        if parity is not None:
            if parity not in pmodes:
                valid = ", ".join(str(pm) for pm in sorted(pmodes))
                raise ValueError("Valid parity modes are " + valid)
            self.parity = parity

        if baudrate is not None:
            if baudrate not in brates:
                brs = sorted(brates)
                dif = [abs(br - baudrate) for br in brs]
                best = brs[dif.index(min(dif))]
                raise ValueError(
                    "Invalid baudrates, nearest valid is {}".format(best))
            self.baudrate = baudrate

        if self._isFTDI:
            self._setBaudFTDI(self.baudrate)
            self._setLineCodeFTDI(
                bits=self.databits,
                stopbits=sbits[self.stopbits],
                parity=self.parity,
                breaktype=0)
        else:
            linecode = [
                self.baudrate & 0xff,
                (self.baudrate >> 8) & 0xff,
                (self.baudrate >> 16) & 0xff,
                (self.baudrate >> 24) & 0xff,
                sbits[self.stopbits],
                self.parity,
                self.databits]

            txdir = 0           # 0:OUT, 1:IN
            req_type = 1        # 0:std, 1:class, 2:vendor
            recipient = 1       # 0:device, 1:interface, 2:endpoint, 3:other
            req_type = (txdir << 7) + (req_type << 5) + recipient

            wlen = self.device.ctrl_transfer(
                req_type, CDC_CMDS["SET_LINE_CODING"],
                data_or_wLength=linecode)
            log.debug("Linecoding set, {}b sent".format(wlen))

    def _setBaudFTDI(self, baudrate):
        if not self._isFTDI:
            return
        actual_baud, value, ndex = ftdi_to_clkbits(baudrate)
        log.debug("Actual baud: {}, Value 0x{:X}, Index {}".format(
            actual_baud, value, ndex))

        txdir = 0           # 0:OUT, 1:IN
        req_type = 2        # 0:std, 1:class, 2:vendor
        recipient = 0       # 0:device, 1:interface, 2:endpoint, 3:other
        req_type = (txdir << 7) + (req_type << 5) + recipient

        self.device.ctrl_transfer(
            bmRequestType=req_type,
            bRequest=3,
            wValue=value,
            wIndex=ndex,
            data_or_wLength=0)

        log.debug("FTDI Baudrate set to {}".format(actual_baud))

    def _setLineCodeFTDI(self, bits, stopbits, parity, breaktype=0):
        if not self._isFTDI:
            return
        value = bits
        value += parity << 8
        value += stopbits << 11
        value += breaktype << 14

        txdir = 0               # 0:OUT, 1:IN
        req_type = 2            # 0:std, 1:class, 2:vendor
        recipient = 0           # 0:device, 1:interface, 2:endpoint, 3:other
        req_type = (txdir << 7) + (req_type << 5) + recipient

        wlen = self.device.ctrl_transfer(
            bmRequestType=req_type,
            bRequest=4,     # line coding
            wValue=value,
            wIndex=1,
            data_or_wLength=0)
        return wlen

    def _resetFTDI(self):
        """ reset the FTDI device
        """
        if not self._isFTDI:
            return
        txdir = 0           # 0:OUT, 1:IN
        req_type = 2        # 0:std, 1:class, 2:vendor
        recipient = 0       # 0:device, 1:interface, 2:endpoint, 3:other
        req_type = (txdir << 7) + (req_type << 5) + recipient
        self.device.ctrl_transfer(
            bmRequestType=req_type,
            bRequest=0,     # RESET
            wValue=0,       # RESET
            wIndex=1,
            data_or_wLength=0)

    def _flushFTDI(self, rx=True, tx=True):
        """ flush rx / tx buffers for ftdi device
        """
        if not self._isFTDI:
            return
        txdir = 0                               # 0:OUT, 1:IN
        req_type = 2 if self._isFTDI else 1     # 0:std, 1:class, 2:vendor
        # 0:device, 1:interface, 2:endpoint, 3:other
        recipient = 0 if self._isFTDI else 1
        req_type = (txdir << 7) + (req_type << 5) + recipient
        if rx:
            self.device.ctrl_transfer(
                bmRequestType=req_type,
                bRequest=0,     # RESET
                wValue=1,       # PURGE RX
                wIndex=1,       # INTERFACE 1
                data_or_wLength=0)
        if tx:
            self.device.ctrl_transfer(
                bmRequestType=req_type,
                bRequest=0,     # RESET
                wValue=2,       # PURGE TX
                wIndex=1,       # INTERFACE 1
                data_or_wLength=0)

    def getLineCoding(self):
        if self._isFTDI:
            log.warning("FTDI does not support reading baud parameters")
        txdir = 1           # 0:OUT, 1:IN
        req_type = 1        # 0:std, 1:class, 2:vendor
        recipient = 1       # 0:device, 1:interface, 2:endpoint, 3:other
        req_type = (txdir << 7) + (req_type << 5) + recipient

        buf = self.device.ctrl_transfer(bmRequestType=req_type,
                                        bRequest=CDC_CMDS["GET_LINE_CODING"],
                                        wValue=0,
                                        wIndex=0,
                                        data_or_wLength=255,
                                        )
        self.baudrate = buf[0] + (buf[1] << 8) + \
            (buf[2] << 16) + (buf[3] << 24)
        self.stopbits = 1 + (buf[4] / 2.0)
        self.parity = buf[5]
        self.databits = buf[6]
        print("LINE CODING:")
        print("  {0} baud, parity mode {1}".format(self.baudrate, self.parity))
        print(
            "  {0} data bits, {1} stop bits".format(
                self.databits,
                self.stopbits))

    def disconnect(self):
        self._endRx()
        while self._rxthread is not None and self._rxthread.isAlive():
            pass
        usb.util.dispose_resources(self.device)
        if self._rxthread is None:
            log.debug("Rx thread never existed")
        else:
            log.debug("Rx thread is {}".format(
                "alive" if self._rxthread.isAlive() else "dead"))
        attempt = 1
        while attempt < 10:
            try:
                self.device.attach_kernel_driver(0)
                log.debug(
                    "Attach kernal driver on attempt {0}".format(attempt))
                break
            except usb.USBError:
                attempt += 1
                time.sleep(0.1)     # sleep seconds
        if attempt == 10:
            log.error("Could not attach kernal driver")
            

def configLog():
    log = logging.getLogger("vcp_terminal")
    log.setLevel(logging.ERROR)
    if "PYTERMINAL_DEBUG" in os.environ:
        fileHandler = logging.FileHandler("terminal.log")
        log_fmt = logging.Formatter(
            "%(levelname)s %(name)s %(threadName)-10s " +
            "%(funcName)s() %(message)s")
        fileHandler.setFormatter(log_fmt)
        log.addHandler(fileHandler)
    return log

def ftdi_to_clkbits(baudrate):  # from libftdi
    """
    10,27 => divisor = 10000, rate = 300
    88,13 => divisor = 5000, rate = 600
    C4,09 => divisor = 2500, rate = 1200
    E2,04 => divisor = 1250, rate = 2,400
    71,02 => divisor = 625, rate = 4,800
    38,41 => divisor = 312.5, rate = 9,600
    D0,80 => divisor = 208.25, rate = 14406
    9C,80 => divisor = 156, rate = 19,230
    4E,C0 => divisor = 78, rate = 38,461
    34,00 => divisor = 52, rate = 57,692
    1A,00 => divisor = 26, rate = 115,384
    0D,00 => divisor = 13, rate = 230,769
    """
    clk = 48000000
    clk_div = 16
    frac_code = [0, 3, 2, 4, 1, 5, 6, 7]
    actual_baud = 0
    if baudrate >= clk / clk_div:
        encoded_divisor = 0
        actual_baud = (clk // clk_div)
    elif baudrate >= clk / (clk_div + clk_div / 2):
        encoded_divisor = 1
        actual_baud = clk // (clk_div + clk_div // 2)
    elif baudrate >= clk / (2 * clk_div):
        encoded_divisor = 2
        actual_baud = clk // (2 * clk_div)
    else:
        # We divide by 16 to have 3 fractional bits and one bit for rounding
        divisor = clk * 16 // clk_div // baudrate
        best_divisor = (divisor + 1) // 2
        if best_divisor > 0x20000:
            best_divisor = 0x1ffff
        actual_baud = clk * 16 // clk_div // best_divisor
        actual_baud = (actual_baud + 1) // 2
        encoded_divisor = ((best_divisor >> 3) +
                           (frac_code[best_divisor & 0x7] << 14))

    value = encoded_divisor & 0xFFFF
    index = encoded_divisor >> 16
    return actual_baud, value, index
log = configLog()
