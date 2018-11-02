#!/usr/bin/python
import os
import commands
import subprocess

p1=0.8 
p2=0
p3=0.36
p4=0.1
p5=0.2
p6=0.3
p7=0.4
p8=4
cm="./compute ik %f %f %f %f %f %f %f %f" % (p1,p2,p3,p4,p5,p6,p7,p8)
#res=os.system("./compute ik %f %f %f %f %f %f %f %f" % (p1,p2,p3,p4,p5,p6,p7,p8))
res=subprocess.Popen(cm,shell=True,stdout=subprocess.PIPE)
#res=commands.getoutput("'ls |awk '{print $2}''")
#print ("dsafsadfdsafasdf")
print "dsfadasfasf"
