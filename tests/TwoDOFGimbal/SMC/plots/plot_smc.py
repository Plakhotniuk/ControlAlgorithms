from tests.TwoDOFGimbal.plot_template import plot_error

input_filename = '../data/TwoAxisGimbalSMCdist.txt'
controller_name = 'Скользяший режим управления.'
output_filename = 'TwoAxisGimbalSMCdistErr.png'

plot_error(input_filename, controller_name, output_filename)

