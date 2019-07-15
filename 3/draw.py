import numpy as np
import matplotlib.pyplot as plt

# 准备数据 
filename = "points.txt"

y = []

#打开txt文件按行读取lamuda
with open(filename, 'r') as f:
	lines = f.readlines()
	for line in lines:
		
		y.append(line)

#画出来
plt.plot(y)
plt.title('labuda')
plt.show()
