def rad_to_ang_sec(rad):
    return rad * 206264

import numpy as np
import matplotlib.pyplot as plt


def plot_trajectories(input_filepath, controller_name, output_filename=0):
    figure, axis = plt.subplots(2, 1)
    data = np.loadtxt(input_filepath)

    position1 = data[:, 0]
    position2 = data[:, 1]

    position_des1 = data[:, 4]
    position_des2 = data[:, 5]

    times = data[:, 8]
    axis[0].title.set_text(f'Зависимость углов от времени.\n {controller_name}')
    axis[0].plot(times, position1, label=r'Текущий угол 1')
    axis[0].plot(times, position_des1, 'r-', label=f'Целевой угол 1')

    axis[0].grid()
    axis[0].set_ylabel('Угол, рад')
    axis[0].legend()
    axis[1].plot(times, position2, label=r'Текущий угол 2')
    axis[1].plot(times, position_des2, 'r-', label=f'Целевой угол 2')

    axis[1].grid()
    axis[1].set_ylabel('Угол, рад')
    axis[1].set_xlabel('время, c')
    axis[1].legend()
    if output_filename:
        plt.savefig(output_filename)
    plt.show()


def plot_error(input_filepath, controller_name, output_filename=0):
    data = np.loadtxt(input_filepath)

    position = np.sqrt(data[:, 0]**2 + data[:, 1]**2)

    position_des = np.sqrt(data[:, 4]**2 + data[:, 5]**2)

    times = data[:, 8]

    plt.plot(times, rad_to_ang_sec(np.abs(position - position_des)))

    plt.title(f'Зависимость ошибки от времени.\n {controller_name}')
    plt.ylabel('угол, угловые секунды')
    plt.xlabel('время, секунды')
    plt.yscale('log')
    plt.grid()
    if output_filename:
        plt.savefig(output_filename)
    plt.show()
