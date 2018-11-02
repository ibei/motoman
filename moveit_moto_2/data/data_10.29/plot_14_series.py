import matplotlib.pyplot as plt

filename_1 = 'cabbage_shake_3_back'
#~ filename_2 = 'carrot_90_shake_1'

X1,Y1,X2,Y2,X3,Y3,X4,Y4,X5,Y5,X6,Y6,X7,Y7,X8,Y8,X9,Y9,X10,Y10= [],[],[],[],[],[],[],[],[],[],[],[],[],[],[],[],[],[],[],[]
a=10*3
b=10*4
c=10*12
d=10*13
#~ e=10*2
#~ f=10*3
with open(filename_1, 'r') as f1:#1
    lines = f1.readlines()[a:b]#2
    i=0
    for line in lines:#3
        value = [float(s) for s in line.split()]#4
        
        X1.append(i)#5
        Y1.append(value[5])
        i+=1
with open(filename_1, 'r') as f2:#1
    lines = f2.readlines()[a+30:b+30]#2
    i=0
    for line in lines:#3
        value = [float(s) for s in line.split()]#4
        
        X2.append(i)#5
        Y2.append(value[5])
        i+=1
#~ with open(filename_1, 'r') as f3:#1
    #~ lines = f3.readlines()[a+20:b+20]#2
    #~ i=0
    #~ for line in lines:#3
        #~ value = [float(s) for s in line.split()]#4
        #~ 
        #~ X3.append(i)#5
        #~ Y3.append(value[5])
        #~ i+=1
#~ with open(filename_2, 'r') as f3:#1
    #~ lines = f3.readlines()[a+10:b+10]#2
    #~ i=0
    #~ for line in lines:#3
        #~ value = [float(s) for s in line.split()]#4
        #~ 
        #~ X3.append(i)#5
        #~ Y3.append(value[5])
        #~ i+=1


#~ with open(filename_2, 'r') as f3:#1
    #~ lines = f3.readlines()[a:b]#2
    #~ i=0
    #~ for line in lines:#3
        #~ value = [float(s) for s in line.split()]#4
        #~ 
        #~ X3.append(i)#5
        #~ Y3.append(value[5])
        #~ i+=1
#~ with open(filename_2, 'r') as f4:#1
    #~ lines = f4.readlines()[c:d]#2
    #~ i=0
    #~ for line in lines:#3
        #~ value = [float(s) for s in line.split()]#4
        #~ 
        #~ X4.append(i)#5
        #~ Y4.append(value[5])
        #~ i+=1

plt.plot(X1,Y1,'b--',label='degree_40_amp_0.15')
plt.plot(X2,Y2,'r--',label='degree_50_amp_0.18')
#~ plt.plot(X3,Y3,'m--',label='amp_0.18')
#~ plt.plot(X4,Y4,'m--',label='shake_90')
#~ plt.plot(X3,Y3,'b-')
#~ plt.plot(X4,Y4,'b-',label='amp_0.18')
#~ plt.plot(X3,Y3,'m--',label='amp_0.15')
plt.xlabel('shaking times')
plt.ylabel('amount_ratio')
plt.legend()
plt.grid(True)
plt.xlim((0.0,12))
plt.ylim((0.0,0.20))
plt.show()


