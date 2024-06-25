import numpy as np


def compute_ise(data):
    position = np.sqrt(data[:, 0]**2 + data[:, 1]**2)
    position_des = np.sqrt(data[:, 4]**2 + data[:, 5]**2)
    error_array = np.abs(position - position_des)

    dt = 0.001

    error_array_squared = error_array * error_array

    return dt * np.sum(error_array_squared)


def compute_iae(data):
    position = np.sqrt(data[:, 0]**2 + data[:, 1]**2)
    position_des = np.sqrt(data[:, 4]**2 + data[:, 5]**2)
    error_array = np.abs(position - position_des)

    dt = 0.001

    return dt * np.sum(error_array)

def compute_itae(data):
    times = data[:, 8]
    position = np.sqrt(data[:, 0]**2 + data[:, 1]**2)
    position_des = np.sqrt(data[:, 4]**2 + data[:, 5]**2)
    error_array = np.abs(position - position_des)

    dt = 0.001

    error_array_w = error_array * times

    return dt * np.sum(error_array_w)


def moving_average(x, w):
    return np.convolve(x, np.ones(w), 'valid') / w


def compute_settling_time(data):
    times = data[:, 8]
    error_array = np.sqrt((data[:, 0] - data[:, 4])**2 + (data[:, 1] - data[:, 5])**2)

    error_average = moving_average(error_array, 1000)
    indexes = np.where(error_average <= error_array[0]*0.05)[0]

    return times[indexes[0]]


def compute_steady_state_error(data):
    error_array = np.sqrt((data[:, 0] - data[:, 4])**2 + (data[:, 1] - data[:, 5])**2)

    return np.mean(error_array[-100:])


data_smc = np.loadtxt('SMC/data/TwoAxisGimbalSMCRealDynamics.txt')
data_pd = np.loadtxt('PD/data/TwoAxisGimbalPDRealDynamics.txt')
data_cascade_pi = np.loadtxt('CascadePI/data/TwoAxisGimbalCascadePIRealDynamics.txt')

print('Интеграл квадрата ошибки | Интеграл модуля ошибки | Интеграл модуля взвешенной ошибки | Время достижения окрестности 5%, | Ошибка установившегося режима')
print('SMC: ', compute_ise(data_smc), " | ", compute_iae(data_smc), " | ", compute_itae(data_smc), " | ", compute_settling_time(data_smc), " | ", compute_steady_state_error(data_smc))

print('PD: ', compute_ise(data_pd), " | ", compute_iae(data_pd), " | ", compute_itae(data_pd), " | ", compute_settling_time(data_pd), " | ", compute_steady_state_error(data_pd))

print('Cascade PI: ', compute_ise(data_cascade_pi), " | ", compute_iae(data_cascade_pi), " | ", compute_itae(data_cascade_pi), " | ", compute_settling_time(data_cascade_pi), " | ", compute_steady_state_error(data_cascade_pi))
