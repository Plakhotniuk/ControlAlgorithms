import numpy as np
import matplotlib.pyplot as plt

filenames = ['SMC/data/TwoAxisGimbalSMCNoiseMetrics.txt',
             'PD/data/TwoAxisGimbalPDNoiseMetrics.txt',
             'CascadePI/data/TwoAxisGimbalCascadePINoiseMetrics.txt']

names = ['Скользящий режим', 'ПД-регулятор', 'Каскадный режим']

def plot_metric(filename, name):
    data = np.loadtxt(filename)

    noise = data[:, 0]

    # ise_metric = data[:, 1]

    # iae_metric = data[:, 2]

    itae_metric = data[:, 3]

    plt.plot(noise, itae_metric, label=name)


for i in range(3):
    plot_metric(filenames[i], names[i])

plt.ylabel('ITAE')
plt.xlabel('стандартное отклонение шума, рад')
plt.legend()
plt.grid()
plt.show()

