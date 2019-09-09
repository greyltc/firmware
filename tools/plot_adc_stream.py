#!/usr/bin/env python

# cool gnome tip for making the bell visual:
# gsettings set org.gnome.desktop.wm.preferences visual-bell true
# gsettings set org.gnome.desktop.wm.preferences visual-bell-type fullscreen-flash

from telnetlib import Telnet
import numpy
import matplotlib.pyplot as plt
import socket
import time

HOST = "WIZnet111785.desk_lan"
PROMPT = b'>>> '  # the firmware's prompt
EOL = b'\r\n'  # client-->server transmission line ending the firmware expects
ADC_BITS = 24
ADC_VREF = 2.048  # volts
ONE_LSB = ADC_VREF/(2**(ADC_BITS-1))  # volts per count
dt = 1036/2.048e6  # sample period


def sock_read_until(s, match):
    matched = False
    buffer = bytes([])
    while (matched is False):
        try:
            buffer = buffer + s.recv(1)
        except socket.timeout:
            break
        if buffer.endswith(match):
            matched = True
    return buffer


class MyTelnet(Telnet):
    def read_response(self):
        return self.read_until(PROMPT).rstrip(PROMPT).decode().strip()

    def send_cmd(self, cmd):
        return self.write(cmd.encode()+EOL)


with MyTelnet(HOST) as tn:
    # get the welcome message
    welcome_message = tn.read_response()
    print(f"Connected with welcome message: {welcome_message}")

    # test a command (ask for virmware revision)
    # tn.write(b'v'+EOL)
    tn.send_cmd('v')
    version_message = tn.read_response()
    print(f"Got version request response: {version_message}")

    tn.send_cmd('stream')  # send the capture command
    tn.read_until(b'interval...')  # get the stream start message
    print('\aStreaming waveform...', end='', flush=True)

    raw_stream = sock_read_until(tn.sock, b'ADC streaming complete.').strip(b'ADC streaming complete.')

    print('done!')
    leftover = tn.read_response()  # get the prompt

    # close/cleanup the connection
    tn.send_cmd('close')  # not strictly needed

sam_len = 4  # number of bytes per sample
raw_samples = [raw_stream[i*sam_len:i*sam_len+sam_len] for i in range(len(raw_stream)//sam_len)]
counts = [int.from_bytes(raw_samples[i][1:4], byteorder='big', signed=True) for i in range(len(raw_samples))]
diff = numpy.array([int(raw_samples[i][0]) for i in range(len(raw_samples))])
misses = sum(diff-1)
if misses == 0:
    print("No samples missed!")
else:
    print(f"WARNING: {misses} samples missed!")

v = numpy.array(counts) * ONE_LSB
t = (numpy.cumsum(diff)-1) * dt
plt.plot(t, v, marker='.')
plt.ylabel('Voltage [V]')
plt.xlabel('Time [s]')
plt.show()
