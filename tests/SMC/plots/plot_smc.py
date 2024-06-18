import numpy as np
import matplotlib.pyplot as plt


def rad_to_ang_sec(rad):
    return rad * 206264


def plot_data(filepath):
    data = np.loadtxt(filepath)

    position = np.sqrt(data[:, 0]**2 + data[:, 1]**2)

    position_des = np.sqrt(data[:, 4]**2 + data[:, 5]**2)

    times = data[:, 8]

    plt.plot(times, rad_to_ang_sec(np.abs(position - position_des)))


# plot_data('../data/TwoAxisGimbalSMC.txt')
plot_data('../data/TwoAxisGimbalSMC2.txt')
plt.title('Зависимость ошибки от времени.\n Скользяший режим управления.')
plt.ylabel('угол, угловые секунды')
plt.xlabel('время, секунды')
plt.yscale('log')
plt.grid()
plt.savefig('two_axis_gimbal_smc.png')
plt.show()
