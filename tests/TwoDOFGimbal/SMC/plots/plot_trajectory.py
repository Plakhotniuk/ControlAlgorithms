from tests.TwoDOFGimbal.plot_template import plot_trajectories


input_filename = '../data/TwoAxisGimbalSMCdynamic.txt'
controller_name = 'Скользящий режим управления.'
output_filename = 'TwoAxisGimbalSMCtrajDynamic.png'

plot_trajectories(input_filename, 0, output_filename)
