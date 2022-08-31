#!/bin/bash
sudo socat pty,link=/dev/ttyVUSB1,raw,group-late=dialout,mode=660 tcp:192.168.2.2:8888