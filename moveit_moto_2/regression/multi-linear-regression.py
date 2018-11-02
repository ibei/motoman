from sklearn.linear_model import LinearRegression
from sklearn.metrics import mean_squared_error
import matplotlib.pyplot as plt
from scipy.optimize import fmin
import numpy as np

from sklearn.model_selection import train_test_split
from sklearn.metrics import mean_squared_error, r2_score

data=np.loadtxt("carrot_h_1.2_proc")
print data.shape
x_list,x_test_1=[],[]
y_list,y_test_1=[],[]
#~ for m in range(27,30,1):
	#~ x_test_1.append([data[m][0],data[m][1],data[m][2],data[m][3],data[m][4]])
	#~ y_test_1.append((data[m][5]+data[m][6]-0.023)**2)
	
for i in range(data.shape[0]):
	x_list.append([data[i][0],data[i][1],data[i][2],data[i][3],data[i][4]])
	#objective func :  (e+v-0.023)**2
	y_list.append((data[i][5]+data[i][6]-0.023)**2)
x_test=np.array(x_test_1)
y_test=np.array(x_test_1)
x=np.array(x_list)
y=np.array(y_list)

Xtrain, Xtest, Ytrain, Ytest = train_test_split(x, y, test_size=0.9, random_state=0)

plt.plot(Xtest[:,0], Ytest, '.')
plt.show()
#~ reg = LinearRegression().fit(x, y)
#~ print reg.score(x, y)
#~ print reg.coef_

#~ print mean_squared_error(y_test,reg.predict(x_test))
#~ print reg.score(x_test,y_test)
#~ print reg.score(x,y)
#~ plt.scatter(x,reg.predict(x),color ='green')
#~ plt.plot(x ,reg.predict(x) ,color='red',linewidth =3)
#~ plt.show()
