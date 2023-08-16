#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np

x1=2
x2=-1
x3=3
y1=3
y2=-1
y3=2

plt.figure(1)
plt.scatter(x1,y1,color='blue')
plt.arrow(x1,y1,0.25,0,ec="green",head_width=0.1)
x=np.array([x2,x3])
y=np.array([y2,y3])
plt.scatter(x,y,color='red')
plt.arrow(x2,y2,-0.25,0,ec="green",head_width=0.1)
plt.arrow(x3,y3,0,0.25,ec="green",head_width=0.1)
plt.title("Robot 1")
plt.grid()
plt.xlim([-5,5])
plt.ylim([-5,5])
# plt.show()

x1=2
x2=-1
x3=3
y1=3
y2=-1
y3=2


plt.figure(2)
plt.scatter(x2,y2,color='blue')
plt.arrow(x1,y1,0.25,0,ec="green",head_width=0.1)
x=np.array([x1,x3])
y=np.array([y1,y3])
plt.scatter(x,y,color='red')
plt.arrow(x2,y2,-0.25,0,ec="green",head_width=0.1)
plt.arrow(x3,y3,0,0.25,ec="green",head_width=0.1)
plt.title("Robot 2")
plt.grid()
plt.xlim([-5,5])
plt.ylim([-5,5])
# plt.show()

x1=2
x2=-1
x3=3
y1=3
y2=-1
y3=2

plt.figure(3)
plt.scatter(x3,y3,color='blue')
plt.arrow(x1,y1,0.25,0,ec="green",head_width=0.1)
x=np.array([x2,x1])
y=np.array([y2,y1])
plt.scatter(x,y,color='red')
plt.arrow(x2,y2,-0.25,0,ec="green",head_width=0.1)
plt.arrow(x3,y3,0,0.25,ec="green",head_width=0.1)
plt.title("Robot 3")
plt.grid()
plt.xlim([-5,5])
plt.ylim([-5,5])
plt.show()
