import numpy as np
import matplotlib.pyplot as plt
import math

heading = np.genfromtxt('heading.csv', delimiter=',')
print heading
headWrap = heading
for ii in range(heading.shape[0]-1):
    pass
    #headWrap[ii+1] = heading[ii] + np.unwrap(heading[ii+1]-heading[ii])
    #print heading[ii+1]
plt.plot(np.unwrap(heading) - heading[0])

plt.show()
