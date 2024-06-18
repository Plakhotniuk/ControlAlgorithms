import numpy as np
import matplotlib.pyplot as plt

data = np.loadtxt('../data/TwoAxisGimbalSMC.txt')


position = np.sqrt(data[:, 0]**2 + data[:, 1]**2)

position_des = np.sqrt(data[:, 4]**2 + data[:, 5]**2)

times = data[:, 8]

plt.plot(times, position - position_des)

plt.grid()
plt.show()
