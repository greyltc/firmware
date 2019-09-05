#!/usr/bin/env python

from telnetlib import Telnet

HOST="WIZnet111785.desk_lan"
PROMPT = b'>>> ' # the firmware's prompt
EOL=b'\r\n' # client --> server transmission line ending the firmware expects

#def send_cmd(tn, cmd):
#  tn.write(cmd.encode()+EOL)

with Telnet(HOST) as tn:
  def send_cmd(tn, cmd):
    tn.write(cmd.encode()+EOL)

  # get the welcome message
  welcome_message = tn.read_until(PROMPT).rstrip(PROMPT).strip().decode()
  print(f"Connected with welcome message: {welcome_message.strip()}")

  # test a command (ask for virmware revision)
  #tn.write(b'v'+EOL)
  send_cmd(tn,'v')
  version_message = tn.read_until(PROMPT).rstrip(PROMPT).strip().decode()
  print(f"Got version request response: {version_message.strip()}")

  # send the capture command



  # close/cleanup the connection
  tn.write(b'close'+EOL)  # not strictly needed
