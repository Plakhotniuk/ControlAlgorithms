import numpy as np
import matplotlib.pyplot as plt


def rad_to_ang_sec(rad):
    return rad * 206264


figure, axis = plt.subplots(2, 1)


def plot_data(filepath):
    data = np.loadtxt(filepath)

    position1 = data[:, 0]
    position2 = data[:, 1]

    position_des1 = data[:, 4]
    position_des2 = data[:, 5]

    times = data[:, 8]
    axis[0].title.set_text('Значения углов во времени.\n Каскадный ПИ регулятор.')
    axis[0].plot(times, position1, 'r-', label=f'Целевой угол 1')
    axis[0].plot(times, position_des1, label=r'Текущий угол 1')
    axis[0].grid()
    axis[0].set_ylabel('Угол, рад')
    axis[0].legend()

    axis[1].plot(times, position2, 'r-', label=f'Целевой угол 2')
    axis[1].plot(times, position_des2, label=r'Текущий угол 2')
    axis[1].grid()
    axis[1].set_ylabel('Угол, рад')
    axis[1].set_xlabel('время, c')
    axis[1].legend()


plot_data('../data/TwoAxisGimbalCascadePIdist.txt')
# plt.savefig('two_axis_gimbal_cascade_pi_traj.png')
plt.show()
