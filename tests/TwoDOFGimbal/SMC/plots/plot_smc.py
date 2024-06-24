from tests.TwoDOFGimbal.plot_template import plot_error

input_filename = '../data/TwoAxisGimbalSMCdynamic.txt'
controller_name = 'Скользяший режим управления.'
output_filename = 'TwoAxisGimbalSMCerrorDynamic.png'

plot_error(input_filename, 0, output_filename)

