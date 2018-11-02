import matplotlib.pyplot as plt

filename_1 = 'cucumber_shake_a'
filename_2 = 'cucumber_shake_b'
filename_3 = 'cucumber_shake_a_b'
#~ filename_2 = 'cucumber_tip_proc'
#~ filename_1 = 'pepper_tipangle_60.0_shaking_freq_2.0_1'
#~ filename_2 = 'pepper_tipangle_60.0_shaking_freq_2.25_1'
#~ filename_3 = 'peppe:r_tipangle_60.0_shaking_freq_2.5_1'
#~ filename_4 = 'pepper_tipangle_60.0_shaking_freq_2.75_1'
#filename_5= 'r_d_f3.5_a0.1_p-times5_bt1.txt'

X1,Y1,X2,Y2,X3,Y3,X4,Y4,X5,Y5,X6,Y6,X7,Y7,X8,Y8,X9,Y9,X10,Y10= [],[],[],[],[],[],[],[],[],[],[],[],[],[],[],[],[],[],[],[]

with open(filename_1, 'r') as f1:#1
    lines = f1.readlines()[48:52]#2
    for line in lines:#3
        #~ print line
        value = [float(s) for s in line.split()]#4
        X1.append(value[4])#5
        Y1.append(value[5])
with open(filename_2, 'r') as f2:#1
    lines = f2.readlines()[48:52]#2
    for line in lines:#3
        #~ print line
        value = [float(s) for s in line.split()]#4
        X2.append(value[4])#5
        Y2.append(value[5])
with open(filename_3, 'r') as f3:#1
    lines = f3.readlines()[48:52]#2
    for line in lines:#3
        #~ print line
        value = [float(s) for s in line.split()]#4
        X3.append(value[4])#5
        Y3.append(value[5])
        
#~ with open(filename_1, 'r') as f2:#1
    #~ lines = f2.readlines()[4:8]#2
    #~ for line in lines:#3

        #~ value = [float(s) for s in line.split()]#4
        #~ X2.append(value[4])#5
        #~ Y2.append(value[5])

#~ with open(filename_1, 'r') as f2:#1
    #~ lines = f2.readlines()[21:24]#2
    #~ for line in lines:#3
        #~ value = [float(s) for s in line.split()]#4
        #~ X2.append(value[4])#5
        #~ Y2.append(value[5])
        
#~ with open(filename_1, 'r') as f3:#1
    #~ lines = f3.readlines()[8:12]#2
    #~ for line in lines:#3
        #~ value = [float(s) for s in line.split()]#4
        #~ X3.append(value[4])#5
        #~ Y3.append(value[5])
#~ with open(filename_1, 'r') as f4:#1
    #~ lines = f4.readlines()[12:16]#2
    #~ for line in lines:#3
        #~ value = [float(s) for s in line.split()]#4
        #~ X4.append(value[4])#5
        #~ Y4.append(value[5])
#~ with open(filename_1, 'r') as f5:#1
    #~ lines = f5.readlines()[30:33]#2
    #~ for line in lines:#3
        #~ value = [float(s) for s in line.split()]#4
        #~ X5.append(value[4])#5
        #~ Y5.append(value[5])
#~ with open(filename_1, 'r') as f6:#1
    #~ lines = f6.readlines()[33:36]#2
    #~ for line in lines:#3
        #~ value = [float(s) for s in line.split()]#4
        #~ X6.append(value[4])#5
        #~ Y6.append(value[5])
        
#~ with open(filename_3, 'r') as f3:#1
    #~ lines = f3.readlines()#2
    #~ for line in lines:#3
        #~ value = [float(s) for s in line.split()]#4
        #~ X3.append(value[0])#5
        #~ Y3.append(value[1])

#t=[0.135]*(int(X[-1])+1)
	#plt.plot(X,t)
#plt.plot(X, Y,'g^',X,t,'r--')
#plt.plot(X, Y,'b--',X,t,'r--')
#~ plt.plot(X1, Y1,'b--')
#~ plt.plot(X2, Y2,'r--')
#~ plt.plot(X3, Y3,'g--')
#~ plt.plot(X4, Y4,'k--')

plt.plot(X1,Y1,'b--',label='shake_a')
plt.plot(X2,Y2,'r--',label='shake_b')
plt.plot(X3,Y3,'y--',label='shake_a_b')
#~ plt.plot(X4,Y4,'m--',label='freq_3.0')
#~ plt.plot(X1,Y2,'r--',label='Variance')
#~ plt.plot(X3,Y3,'g--',label='shake_axis_0_freq_2.75')
#~ plt.plot(X4,Y4,'k-',label='shake_axis_90_freq_2.25')
#~ plt.plot(X5,Y5,'y-',label='shake_axis_90_freq_2.5')
#~ plt.plot(X6,Y6,'m-',label='shake_axis_90_freq_2.75')


plt.xlabel('shake amplitude/m')
plt.ylabel('amount_ratio')

plt.legend()
#plt.text(60, .15, r'$\mu=100,\ \sigma=15$')
#plt.text(2800, .125, 'target_ratio',fontsize=12)
#plt.text(2800, .004, 'realtime_ratio',fontsize=12)
#plt.text(2800, .125, 'target_ratio',fontsize=12)
#plt.axis([40, 160, 0, 0.03])

plt.grid(True)
#~ plt.ylim((0.0,0.15))
plt.show()


