import matplotlib.pyplot as plt
import numpy as np

a=np.loadtxt('moyashi_tip_1')
dfile=open("moyashi_tip_1_proc","a")
#~ print a[0][0]
for i in range(9):
	sum_e=0
	sum_v=0
	for j in range(6):
		sum_e+=a[i*6+j][1]
		h=a[i*6+j][0]
		#~ ang=a[i*5+j][1]
		#~ s_a=a[i*5+j][2]
		#~ f=a[i*5+j][3]
		#~ amp=a[i*5+j][4]
	e=sum_e/6.0
	for m in range(6):
		sum_v+=(a[i*5+m][1]*100-e*100)**2
	v=sum_v/6.0
	#~ dfile.write("%f %f %f %f %f %f %f\r\n" % (h,ang,s_a,f,amp,e,v))
	dfile.write("%f %f %f\r\n" % (h,e,v))
	#~ afile.close()
			
	
		
		

#~ filename_1 = 'carrot_h_0.12'
#~ filename_1 = 'pepper_tipangle_60.0_shaking_freq_2.0_1'
#~ filename_2 = 'pepper_tipangle_60.0_shaking_freq_2.25_1'
#~ filename_3 = 'pepper_tipangle_60.0_shaking_freq_2.5_1'
#~ filename_4 = 'pepper_tipangle_60.0_shaking_freq_2.75_1'
#filename_5= 'r_d_f3.5_a0.1_p-times5_bt1.txt'

#~ X1,Y1,X2,Y2,X3,Y3,X4,Y4,X5,Y5,X6,Y6,X7,Y7,X8,Y8,X9,Y9,X10,Y10= [],[],[],[],[],[],[],[],[],[],[],[],[],[],[],[],[],[],[],[]
#~ height,angle,s_axis,freq,amp,ratio_e,ratio_v=[],[],[],[],[],[],[]
#~ dfile=open("carrot_h_1.2_proc","a")
#~ with open(filename_1, 'r') as f1:#1
	#~ for i in range(0,175,5):
		#~ ratio_1=0
		#~ lines = f1.readlines()[i:(i+5)]#2
		#~ ratio=[]
		#~ for line in lines:#3
			#~ value = [float(s) for s in line.split()]#4
			#~ height_1=value[0]#5
			#~ angle_1=value[1]
			#~ s_axis_1=value[2]
			#~ freq_1=value[3]
			#~ amp_1=value[4]
			#~ ratio_1+=value[5]
			#~ ratio.append(value[5])
		#~ height.append(height_1)
		#~ angle.append(angle_1)
		#~ s_axis.append(s_axis_1)
		#~ freq.append(freq_1)
		#~ amp.append(amp_1)
		#~ e=ratio_1/5.0
		#~ ratio_e.append(e)
		#~ v=0.0
		#~ for n in range(5):
			#~ v+=(ratio[n]-e)**2.0
		#~ ratio_v=v/5.0
		#~ dfile.write("%f %f %f %f %f %f %f\r\n" % (height_1,angle_1,s_axis_1,freq_1,amp_1,e,ratio_v))			

#~ with open(filename_2, 'r') as f2:#1
    #~ lines = f2.readlines()#2
    #~ for line in lines:#3
        #~ value = [float(s) for s in line.split()]#4
        #~ X2.append(value[0])#5
        #~ Y2.append(value[1])
#~ with open(filename_3, 'r') as f3:#1
    #~ lines = f3.readlines()#2
    #~ for line in lines:#3
        #~ value = [float(s) for s in line.split()]#4
        #~ X3.append(value[0])#5
        #~ Y3.append(value[1])
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
    

#t=[0.135]*(int(X[-1])+1)
	#plt.plot(X,t)
#plt.plot(X, Y,'g^',X,t,'r--')
#plt.plot(X, Y,'b--',X,t,'r--')
#~ plt.plot(X1, Y1,'b--')
#~ plt.plot(X2, Y2,'r--')
#~ plt.plot(X3, Y3,'g--')
#~ plt.plot(X4, Y4,'k--')

#~ plt.plot(X1, Y1,'b--',label='freq 2.0')
#~ plt.plot(X2,Y2,'r--',label='freq 2.25')
#~ plt.plot(X3,Y3,'g--',label='freq 2.5')
#~ plt.plot(X4,Y4,'k--',label='freq 2.75')
#~ plt.plot(X5,Y5,'y--',lable='freq 2.0')

#~ plt.xlabel('Shaking Amplitude/m')
#~ plt.xlabel('shake amplitude/m')
#~ plt.ylabel('amount_ratio')
#plt.title('Shaking tomato')
#plt.title('Shaking radish')
#~ plt.title('Shaking r_amp_f2.75_p-times5_bt1_1')
#~ plt.legend()
#plt.text(60, .15, r'$\mu=100,\ \sigma=15$')
#plt.text(2800, .125, 'target_ratio',fontsize=12)
#plt.text(2800, .004, 'realtime_ratio',fontsize=12)
#plt.text(2800, .125, 'target_ratio',fontsize=12)
#plt.axis([40, 160, 0, 0.03])
#~ plt.grid(True)
#~ plt.ylim((0.0,0.10))
#~ plt.show()


