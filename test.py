import numpy as np

data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
print(data[0][0])