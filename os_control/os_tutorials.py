# OS library for opening a new terminal and executing a command
import os
po = os.system("gnome-terminal -e 'bash -c \"roscore; exec bash\"'")



