import csv
import time
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D
from scipy import sparse
import sys


'''with open('output.csv','rb') as csvfile:
    reader = csv.reader(csvfile,delimiter=',')
    #for row in reader:
        #pass
        #print ','.join(row)
'''
if (len(sys.argv) > 1):
   filename = sys.argv[1]
else:
   filename = 'output.csv'
# read in data
print filename
data = np.genfromtxt(filename, dtype=float,skip_header=1, delimiter=',',usecols=(0,1,2,3,4))
print data
# x y psi bias
time = data[:,0]
x = data[:,1]
y = data[:,2]
psi = data[:,3]
bias = data[:,4]

plt.figure(1)
plt.subplot(311)
plt.scatter( x,y, c='r', alpha=0.5)
plt.subplot(312)
plt.scatter(time, psi, c='g', alpha=0.5)
plt.subplot(313)
plt.scatter(time, bias, c='y', alpha=0.5)
print time[-1]
plt.figure(2)
plt.scatter(time, bias, c='y', alpha=0.5)
#plt.scatter(time,-0.0006788*np.sin((np.pi/time[-1])*time)) # sim data
plt.scatter(time,-0.00008788*np.sin((np.pi/time[-1])*time)) # real data
plt.xlabel("time")
plt.ylabel("Heading Estimate, rad/s")
plt.title("Iceberg rotation Estimate")
plt.axis([-30, time[-1], -.002, .002])
#plt.scatter(time,0.0001*np.ones(time.shape),c='r')
plt.show()

