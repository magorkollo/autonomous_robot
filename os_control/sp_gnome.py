# subprocess library used for opening a gnome terminal with popen
import time
import subprocess as sp
p1 = sp.Popen('ls', stdin = sp.PIPE, stderr = sp.PIPE)
rc = p1.wait()
print(p1.pid)
try:
    p1.stdin.write('ls\n'.encode())
    p1.stdin.flush()
except BrokenPipeError:
    print(p1.stderr)
time.sleep(1)
# print("new command")
# p1.communicate(input="ls")
# p1.kill()