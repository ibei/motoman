import matplotlib.pyplot as plt

#~ filename_1 = 'carrot_pre-angle_0.0'
filename_1 = 'carrot_h_0.1_ta_70.0_shaking_freq_2.0_2'
filename_2 = 'carrot_h_0.1_ta_70.0_shaking_freq_2.25_2'
filename_3 = 'carrot_h_0.1_ta_70.0_shaking_freq_2.5_2'
#~ filename_4 = 'carrot_h_0.1_ta_70.0_shaking_freq_2.75_2'
#filename_5= 'r_d_f3.5_a0.1_p-times5_bt1.txt'

X1,Y1,X2,Y2,X3,Y3,X4,Y4,X5,Y5,X6,Y6,X7,Y7,X8,Y8,X9,Y9,X10,Y10= [],[],[],[],[],[],[],[],[],[],[],[],[],[],[],[],[],[],[],[]

with open(filename_1, 'r') as f1:#1
    lines = f1.readlines()#2
    for line in lines:#3
        value = [float(s) for s in line.split()]#4
        X1.append(value[0])#5
        Y1.append(value[1])
with open(filename_2, 'r') as f2:#1
    lines = f2.readlines()#2
    for line in lines:#3
        value = [float(s) for s in line.split()]#4
        X2.append(value[0])#5
        Y2.append(value[1])
with open(filename_3, 'r') as f3:#1
    lines = f3.readlines()#2
    for line in lines:#3
        value = [float(s) for s in line.split()]#4
        X3.append(value[0])#5
        Y3.append(value[1])
#~ with open(filename_4, 'r') as f4:#1
    #~ lines = f4.readlines()#2
    #~ for line in lines:#3
        #~ value = [float(s) for s in line.split()]#4
        #~ X4.append(value[0])#5
        #~ Y4.append(value[1])

#~ with open(filename_1, 'r') as f5:#1
    #~ lines = f5.readlines()[36:45]#2
    #~ for line in lines:#3
        #~ value = [float(s) for s in line.split()]#4
        #~ X5.append(value[0])#5
        #~ Y5.append(value[1])
    
'''     
with open(filename_2, 'r') as f2:#1
    lines = f2.readlines()#2
    for line in lines:#3
        value = [float(s) for s in line.split()]#4
        X2.append(value[0])#5
        Y2.append(value[1])
with open(filename_3, 'r') as f3:#1
    lines = f3.readlines()#2
    for line in lines:#3
        value = [float(s) for s in line.split()]#4
        X3.append(value[0])#5
        Y3.append(value[1])
'''
#t=[0.135]*(int(X[-1])+1)
	#plt.plot(X,t)
#plt.plot(X, Y,'g^',X,t,'r--')
#plt.plot(X, Y,'b--',X,t,'r--')
#~ plt.plot(X1, Y1,'b--')
#~ plt.plot(X2, Y2,'r--')
#~ plt.plot(X3, Y3,'g--')
#~ plt.plot(X4, Y4,'k--')

plt.plot(X1, Y1,'b--',label='freq 2.0')
plt.plot(X2,Y2,'r--',label='freq 2.25')
plt.plot(X3,Y3,'g--',label='freq 2.5')
#~ plt.plot(X4,Y4,'k--',label='freq 2.75')
#~ plt.plot(X5,Y5,'y--',lable='freq 2.0')

#~ plt.xlabel('Shaking Amplitude/m')
plt.xlabel('shake amplitude/m')
plt.ylabel('amount_ratio')
#plt.title('Shaking tomato')
#plt.title('Shaking radish')
#~ plt.title('Shaking r_amp_f2.75_p-times5_bt1_1')
plt.legend()
#plt.text(60, .15, r'$\mu=100,\ \sigma=15$')
#plt.text(2800, .125, 'target_ratio',fontsize=12)
#plt.text(2800, .004, 'realtime_ratio',fontsize=12)
#plt.text(2800, .125, 'target_ratio',fontsize=12)
#plt.axis([40, 160, 0, 0.03])
plt.grid(True)
plt.ylim((0.0,0.10))
plt.show()


