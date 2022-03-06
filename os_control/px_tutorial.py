import pexpect 

child = pexpect.spawn('ls -la')
fout = open('mylog.txt','wb')
child.logfile = fout
