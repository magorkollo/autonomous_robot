import subprocess as sp
# basic subprocess functions 
print(dir(sp))

cmd = 'ping -c 5 google.com'
p1 = sp.Popen(cmd, 
        shell = True,
        stdout = sp.PIPE,
        stdin = sp.PIPE)
rc = p1.wait()

out, err = p1.communicate()

print('Return Code:',rc,'\n')
print('output is: \n', out.decode())
print('error is: \n', err)