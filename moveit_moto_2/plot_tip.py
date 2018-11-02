import matplotlib.pyplot as plt

filename_1 = 'cucumber_tip_proc'
filename_2 = 'carrot_tip_1_proc'
filename_3 = 'cabbage_tip_proc'
#~ filename_4 = 'moyashi_tip_1_proc'
#~ filename_3 = 'cucumber_tip_proc'
#~ filename_4 = 'cabbage_tip_proc'
#~ filename_2 = 'cucumber_tip_proc'
#~ filename_1 = 'pepper_tipangle_60.0_shaking_freq_2.0_1'
#~ filename_2 = 'pepper_tipangle_60.0_shaking_freq_2.25_1'
#~ filename_3 = 'peppe:r_tipangle_60.0_shaking_freq_2.5_1'
#~ filename_4 = 'pepper_tipangle_60.0_shaking_freq_2.75_1'
#filename_5= 'r_d_f3.5_a0.1_p-times5_bt1.txt'

X1,Y1,X2,Y2,X3,Y3,X4,Y4,X5,Y5,X6,Y6,X7,Y7,X8,Y8,X9,Y9,X10,Y10= [],[],[],[],[],[],[],[],[],[],[],[],[],[],[],[],[],[],[],[]

with open(filename_1, 'r') as f1:#1
    lines = f1.readlines()#2
    for line in lines:#3
        #~ print line
        value = [float(s) for s in line.split()]#4
        X1.append(value[0])#5
        Y1.append(value[1])
with open(filename_2, 'r') as f2:#1
    lines = f2.readlines()#2
    for line in lines:#3
        #~ print line
        value = [float(s) for s in line.split()]#4
        X2.append(value[0])#5
        Y2.append(value[1])
with open(filename_3, 'r') as f3:#1
    lines = f3.readlines()#2
    for line in lines:#3
  #~ 
        value = [float(s) for s in line.split()]#4
        X3.append(value[0])#5
        Y3.append(value[1])
#~ with open(filename_4, 'r') as f4:#1
    #~ lines = f4.readlines()#2
    #~ for line in lines:#3
  
        #~ value = [float(s) for s in line.split()]#4
        #~ X4.append(value[0])#5
        #~ Y4.append(value[1])
        #~ Y5.append(0.025)
#~ with open(filename_4, 'r') as f4:#1
    #~ lines = f4.readlines()#2
    #~ for line in lines:#3

        #~ value = [float(s) for s in line.split()]#4
        #~ X4.append(value[0])#5
        #~ Y4.append(value[1])

plt.plot(X1,Y1,'r--',label='cucumber')
plt.plot(X2,Y2,'b--',label='carrot')
plt.plot(X3,Y3,'m--',label='cabbage')
#~ plt.plot(X4,Y4,'y--',label='moyashi')
#~ plt.plot(X4,Y5,'k--')
#~ plt.plot(X4,Y4,'y--',label='cabbage')
#~ plt.plot(X3,Y3,'g--',label='shake_axis_0_freq_2.75')
#~ plt.plot(X4,Y4,'k-',label='shake_axis_90_freq_2.25')
#~ plt.plot(X5,Y5,'y-',label='shake_axis_90_freq_2.5')
#~ plt.plot(X6,Y6,'m-',label='shake_axis_90_freq_2.75')


plt.xlabel('tipping angle/degree')
plt.ylabel('amount_ratio')

plt.legend()
#~ plt.text(11, .03, 'threshold_ratio',fontsize=10)
#plt.text(2800, .125, 'target_ratio',fontsize=12)
#plt.text(2800, .004, 'realtime_ratio',fontsize=12)
#plt.text(2800, .125, 'target_ratio',fontsize=12)
#plt.axis([40, 160, 0, 0.03])

plt.grid(True)
#~ plt.ylim((0.0,0.15))
plt.show()


