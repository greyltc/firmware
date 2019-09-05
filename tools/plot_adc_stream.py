#!/usr/bin/env python

from telnetlib import Telnet
import numpy
import matplotlib.pyplot as plt

HOST = "WIZnet111785.desk_lan"
PROMPT = b'>>> '  # the firmware's prompt
EOL = b'\r\n'  # client-->server transmission line ending the firmware expects
dt = 1036/2.048e6  # sample period


def send_cmd(self, cmd):
    return self.write(cmd.encode()+EOL)


def read_response(self):
    return self.read_until(PROMPT).rstrip(PROMPT).strip().decode()


Telnet.read_response = read_response
Telnet.send_cmd = send_cmd
with Telnet(HOST) as tn:
    # get the welcome message
    welcome_message = tn.read_response()
    print(f"Connected with welcome message: {welcome_message}")

    # test a command (ask for virmware revision)
    # tn.write(b'v'+EOL)
    tn.send_cmd('v')
    version_message = tn.read_response()
    print(f"Got version request response: {version_message}")

    # send the capture command
    tn.send_cmd('stream')
    print('Capturing waveform...', end='', flush=True)
    tn.read_until(b'interval...')
    tn.read_until(b'back:')
    print('done!')

    print('Reading back waveform data...', end='', flush=True)
    samples = tn.read_until(b'ADC').decode().strip('ADC').strip().split()
    print('done!')
    tn.read_response()  # get the prompt

    # close/cleanup the connection
    tn.send_cmd('close')  # not strictly needed

v = numpy.array([float(x) for x in samples])
t = numpy.array(range(len(samples)))*dt
plt.plot(t, v)
plt.ylabel('Voltage [V]')
plt.xlabel('Time [s]')
plt.show()
