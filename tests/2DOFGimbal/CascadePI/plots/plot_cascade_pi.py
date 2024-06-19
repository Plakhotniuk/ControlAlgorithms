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


plot_data('../data/TwoAxisGimbalCascadePIdist.txt')
plt.title('Зависимость ошибки от времени.\n Каскадный ПИ регулятор.')
plt.ylabel('угол, угловые секунды')
plt.xlabel('время, секунды')
plt.yscale('log')
plt.grid()
# plt.savefig('two_axis_gimbal_cascade_pi.png')
plt.show()
