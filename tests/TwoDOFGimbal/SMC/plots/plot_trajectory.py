from tests.TwoDOFGimbal.plot_template import plot_trajectories


input_filename = '../data/TwoAxisGimbalSMC.txt'
controller_name = 'Скользящий режим управления.'
output_filename = 'TwoAxisGimbalSMCstep.png'

plot_trajectories(input_filename, controller_name, output_filename)
