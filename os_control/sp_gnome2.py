#!/usr/bin/env python
import os
import signal
import subprocess
import time

p = subprocess.Popen(['gnome-terminal', '--disable-factory', '-e', 'roscore'],
                     preexec_fn=os.setpgrp)
time.sleep(5)
p2 = subprocess.Popen(['gnome-terminal', '--disable-factory', '-e', 'pwd'], stdin = subprocess.PIPE,
                     preexec_fn=os.setpgrp)
p2.communicate(input="ls\n".encode())
time.sleep(50)
os.killpg(p.pid, signal.SIGINT)