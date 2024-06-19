from tests.TwoDOFGimbal.plot_template import plot_trajectories


input_filename = '../data/TwoAxisGimbalSMCdist.txt'
controller_name = 'Скользящий режим управления.'
output_filename = 'TwoAxisGimbalSMCdistTraj.png'

plot_trajectories(input_filename, controller_name, output_filename)
