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



data_smc = np.loadtxt('SMC/data/TwoAxisGimbalSMCdynamic.txt')
data_pd = np.loadtxt('PD/data/TwoAxisGimbalPDdynamic.txt')
data_cascade_pi = np.loadtxt('CascadePI/data/TwoAxisGimbalCascadePIdynamic.txt')

print('Интеграл квадрата ошибки | Интеграл модуля ошибки | Интеграл модуля взвешенной ошибки')
print('SMC: ', compute_ise(data_smc), " | ", compute_iae(data_smc), " | ", compute_itae(data_smc))

print('PD: ', compute_ise(data_pd), " | ", compute_iae(data_pd), " | ", compute_itae(data_pd))

print('Cascade PI: ', compute_ise(data_cascade_pi), " | ", compute_iae(data_cascade_pi), " | ", compute_itae(data_cascade_pi))
